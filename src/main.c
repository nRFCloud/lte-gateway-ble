/*
 * Copyright (c) 2016 Nordic Semiconductor ASA
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include <zephyr.h>
#include <arch/cpu.h>
#include <sys/byteorder.h>
#include <logging/log.h>
#include <sys/util.h>

#include <device.h>
#include <init.h>
#include <drivers/uart.h>

#include <net/buf.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/l2cap.h>
#include <bluetooth/hci.h>
#include <bluetooth/buf.h>
#include <bluetooth/hci_raw.h>

#include <nrfx.h>
#include <hal/nrf_power.h>
#include <power/reboot.h>
#include <usb/usb_device.h>
#include <drivers/gpio.h>

#include "usb_uart_bridge.h"

LOG_MODULE_REGISTER(lte_gateway_ble, CONFIG_LTE_GATEWAY_BLE_LOG_LEVEL);

/* #define DEBUG_UART_PINS */
#if defined(DEBUG_UART_PINS)
#define UART0 DT_NODELABEL(uart0)
#define UART0_TX DT_PROP(UART0, tx_pin)
#define UART0_RX DT_PROP(UART0, rx_pin)
#define UART0_RTS DT_PROP(UART0, rts_pin)
#define UART0_CTS DT_PROP(UART0, cts_pin)
#define UART0_SPEED DT_PROP(UART0, current_speed)
#define UART1 DT_NODELABEL(uart1)
#define UART1_TX DT_PROP(UART1, tx_pin)
#define UART1_RX DT_PROP(UART1, rx_pin)
#define UART1_RTS DT_PROP(UART1, rts_pin)
#define UART1_CTS DT_PROP(UART1, cts_pin)
#define UART1_SPEED DT_PROP(UART1, current_speed)
#endif

#if defined(CONFIG_BT_CTLR_TX_BUFFER_SIZE)
#define BT_L2CAP_MTU (CONFIG_BT_CTLR_TX_BUFFER_SIZE - BT_L2CAP_HDR_SIZE)
#else
#define BT_L2CAP_MTU 65 /* 64-byte public key + opcode */
#endif /* CONFIG_BT_CTLR */

/** Data size needed for ACL buffers */
#define BT_BUF_ACL_SIZE BT_L2CAP_BUF_SIZE(BT_L2CAP_MTU)

#if defined(CONFIG_BT_CTLR_TX_BUFFERS)
#define TX_BUF_COUNT CONFIG_BT_CTLR_TX_BUFFERS
#else
#define TX_BUF_COUNT 6
#endif
#define CMD_BUF_SIZE BT_BUF_RX_SIZE

#define H4_CMD 0x01
#define H4_ACL 0x02
#define H4_SCO 0x03
#define H4_EVT 0x04

/* Length of a discard/flush buffer.
 * This is sized to align with a BLE HCI packet:
 * 1 byte H:4 header + 32 bytes ACL/event data
 * Bigger values might overflow the stack since this is declared as a local
 * variable, smaller ones will force the caller to call into discard more
 * often.
 */
#define H4_DISCARD_LEN 33

#define H4_SEND_TIMEOUT 5000

static struct device *hci_uart_dev;
static K_THREAD_STACK_DEFINE(tx_thread_stack, CONFIG_BT_HCI_TX_STACK_SIZE);
static struct k_thread tx_thread_data;

/* HCI command buffers */
NET_BUF_POOL_FIXED_DEFINE(acl_tx_pool, TX_BUF_COUNT, BT_BUF_ACL_SIZE, NULL);

static K_FIFO_DEFINE(tx_queue);

/* RX in terms of bluetooth communication */
static K_FIFO_DEFINE(uart_tx_queue);

NET_BUF_POOL_FIXED_DEFINE(cmd_tx_pool, CONFIG_BT_HCI_CMD_COUNT, CMD_BUF_SIZE,
			  NULL);

static int h4_read(struct device *uart, uint8_t *buf,
		   size_t len, size_t min)
{
	int total = 0;

	while (len) {
		int rx;

		rx = uart_fifo_read(uart, buf, len);
		if (rx == 0) {
			if (total < min) {
				continue;
			}
			break;
		}

		LOG_DBG("read %d remaining %d", rx, len - rx);
		len -= rx;
		total += rx;
		buf += rx;
	}

	return total;
}

static size_t h4_discard(struct device *uart, size_t len)
{
	uint8_t buf[H4_DISCARD_LEN];

	return uart_fifo_read(uart, buf, MIN(len, sizeof(buf)));
}

static struct net_buf *h4_cmd_recv(int *remaining)
{
	struct bt_hci_cmd_hdr hdr;
	struct net_buf *buf;

	/* We can ignore the return value since we pass len == min */
	h4_read(hci_uart_dev, (void *)&hdr, sizeof(hdr), sizeof(hdr));

	*remaining = hdr.param_len;

	buf = net_buf_alloc(&cmd_tx_pool, K_NO_WAIT);
	if (buf) {
		bt_buf_set_type(buf, BT_BUF_CMD);
		net_buf_add_mem(buf, &hdr, sizeof(hdr));
	} else {
		LOG_ERR("No available command buffers!");
	}

	LOG_DBG("hdr.opcode=0x%04x hdr.len=%u",
		hdr.opcode, hdr.param_len);

	return buf;
}

static struct net_buf *h4_acl_recv(int *remaining)
{
	struct bt_hci_acl_hdr hdr;
	struct net_buf *buf;

	/* We can ignore the return value since we pass len == min */
	h4_read(hci_uart_dev, (void *)&hdr, sizeof(hdr), sizeof(hdr));

	buf = net_buf_alloc(&acl_tx_pool, K_NO_WAIT);
	if (buf) {
		bt_buf_set_type(buf, BT_BUF_ACL_OUT);
		net_buf_add_mem(buf, &hdr, sizeof(hdr));
	} else {
		LOG_ERR("No available ACL buffers!");
	}

	*remaining = sys_le16_to_cpu(hdr.len);

	LOG_DBG("len %u", *remaining);

	return buf;
}

static void tx_isr(void)
{
	static struct net_buf *buf;
	int len;

	if (!buf) {
		buf = net_buf_get(&uart_tx_queue, K_NO_WAIT);
		if (!buf) {
			uart_irq_tx_disable(hci_uart_dev);
			return;
		}
	}

	len = uart_fifo_fill(hci_uart_dev, buf->data, buf->len);
	net_buf_pull(buf, len);
	if (!buf->len) {
		net_buf_unref(buf);
		buf = NULL;
	}
}

static void bt_uart_isr(struct device *unused, void *user_data)
{
	static struct net_buf *buf;
	static int remaining;

	ARG_UNUSED(unused);
	ARG_UNUSED(user_data);

	while (uart_irq_update(hci_uart_dev) &&
	       uart_irq_is_pending(hci_uart_dev)) {
		int read;

		if (uart_irq_tx_ready(hci_uart_dev)) {
			tx_isr();
		} else if (!uart_irq_rx_ready(hci_uart_dev)) {
			/* Only the UART TX and RX path are interrupt-enabled */
			LOG_DBG("spurious interrupt");
			break;
		}

		/* Beginning of a new packet */
		if (!remaining) {
			uint8_t type;

			/* Get packet type */
			read = h4_read(hci_uart_dev, &type, sizeof(type), 0);
			if (read != sizeof(type)) {
				LOG_WRN("Unable to read H4 packet type");
				continue;
			}

			switch (type) {
			case H4_CMD:
				buf = h4_cmd_recv(&remaining);
				break;
			case H4_ACL:
				buf = h4_acl_recv(&remaining);
				break;
			default:
				LOG_ERR("Unknown H4 type %u", type);
				return;
			}

			LOG_DBG("need to get %u bytes", remaining);

			if (buf && remaining > net_buf_tailroom(buf)) {
				LOG_ERR("Not enough space in buffer");
				net_buf_unref(buf);
				buf = NULL;
			}
		}

		if (!buf) {
			read = h4_discard(hci_uart_dev, remaining);
			LOG_WRN("Discarded %d bytes", read);
			remaining -= read;
			continue;
		}

		read = h4_read(hci_uart_dev, net_buf_tail(buf), remaining, 0);

		buf->len += read;
		remaining -= read;

		LOG_DBG("received %d bytes", read);

		if (!remaining) {
			LOG_DBG("full packet received");

			/* Put buffer into TX queue, thread will dequeue */
			net_buf_put(&tx_queue, buf);
			buf = NULL;
		}
	}
}

static void tx_thread(void *p1, void *p2, void *p3)
{
	while (1) {
		struct net_buf *buf;
		int err;

		/* Wait until a buffer is available */
		buf = net_buf_get(&tx_queue, K_FOREVER);
		/* Pass buffer to the stack */
		LOG_HEXDUMP_DBG(buf->data, buf->len, "tx_thread");
		LOG_DBG("net_buf_get() returned len %d", buf->len);
		err = bt_send(buf);
		if (err) {
			LOG_ERR("Unable to send (err %d)", err);
			net_buf_unref(buf);
		}

		/* Give other threads a chance to run if tx_queue keeps getting
		 * new data all the time.
		 */
		k_yield();
	}
}

static int h4_send(struct net_buf *buf)
{
	LOG_DBG("buf %p type %u len %u", buf, bt_buf_get_type(buf),
		    buf->len);

	net_buf_put(&uart_tx_queue, buf);
	uart_irq_tx_enable(hci_uart_dev);

	return 0;
}

#if defined(CONFIG_BT_CTLR_ASSERT_HANDLER)
void bt_ctlr_assert_handle(char *file, uint32_t line)
{
	uint32_t len = 0U, pos = 0U;

	/* Disable interrupts, this is unrecoverable */
	(void)irq_lock();

	uart_irq_rx_disable(hci_uart_dev);
	uart_irq_tx_disable(hci_uart_dev);

	if (file) {
		while (file[len] != '\0') {
			if (file[len] == '/') {
				pos = len + 1;
			}
			len++;
		}
		file += pos;
		len -= pos;
	}

	uart_poll_out(hci_uart_dev, H4_EVT);
	/* Vendor-Specific debug event */
	uart_poll_out(hci_uart_dev, 0xff);
	/* 0xAA + strlen + \0 + 32-bit line number */
	uart_poll_out(hci_uart_dev, 1 + len + 1 + 4);
	uart_poll_out(hci_uart_dev, 0xAA);

	if (len) {
		while (*file != '\0') {
			uart_poll_out(hci_uart_dev, *file);
			file++;
		}
		uart_poll_out(hci_uart_dev, 0x00);
	}

	uart_poll_out(hci_uart_dev, line >> 0 & 0xff);
	uart_poll_out(hci_uart_dev, line >> 8 & 0xff);
	uart_poll_out(hci_uart_dev, line >> 16 & 0xff);
	uart_poll_out(hci_uart_dev, line >> 24 & 0xff);

	while (1) {
	}
}
#endif /* CONFIG_BT_CTLR_ASSERT_HANDLER */
static int hci_uart_init(struct device *unused)
{
	LOG_DBG("init hci uart");

	/* Derived from DT's bt-c2h-uart chosen node */
	hci_uart_dev = device_get_binding(CONFIG_BT_CTLR_TO_HOST_UART_DEV_NAME);
	if (!hci_uart_dev) {
		return -EINVAL;
	}

	uart_irq_rx_disable(hci_uart_dev);
	uart_irq_tx_disable(hci_uart_dev);

	uart_irq_callback_set(hci_uart_dev, bt_uart_isr);

	uart_irq_rx_enable(hci_uart_dev);

	return 0;
}

DEVICE_INIT(hci_uart, "hci_uart", &hci_uart_init, NULL, NULL,
	    APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);


void output_string(int id, char *str)
{
	struct serial_dev *sd = &devs[id];
	struct uart_data *rx = k_malloc(sizeof(struct uart_data));

	if (rx) {
		memset(rx, 0, sizeof(rx));
		rx->len = strlen(str);
		strcpy(rx->buffer, str);
		k_fifo_put(sd->fifo, rx);
		k_sem_give(&sd->sem);
	}
}

#define DBG(x) output_string(0, x)

static void log_uart_pins(void)
{
#if defined(DEBUG_UART_PINS)
	struct uart_config config;
	struct device *uart_0_dev = device_get_binding("UART_0");
	struct device *uart_1_dev = device_get_binding("UART_1");

	LOG_INF("UART0 tx:%d, rx:%d, rts:%d, cts:%d, speed:%d",
		UART0_TX, UART0_RX, UART0_RTS, UART0_CTS, UART0_SPEED);
	LOG_INF("UART1 tx:%d, rx:%d, rts:%d, cts:%d, speed:%d",
		UART1_TX, UART1_RX, UART1_RTS, UART1_CTS, UART1_SPEED);
	k_sleep(K_MSEC(50));

	uart_config_get(uart_0_dev, &config);
	LOG_INF("UART0 speed:%u, flow:%d", config.baudrate,
		config.flow_ctrl);
	k_sleep(K_MSEC(50));

	uart_config_get(uart_1_dev, &config);
	LOG_INF("UART1 speed:%u, flow:%d", config.baudrate,
		config.flow_ctrl);

	LOG_INF("Reset pin:%d",
		CONFIG_BOARD_APRICITY_GATEWAY_NRF52840_RESET_PIN);
#endif
}

extern bool ignore_reset;
void main(void)
{
	ignore_reset = false;

	/* incoming events and data from the controller */
	static K_FIFO_DEFINE(rx_queue);
	int err;
	int ret;
	struct serial_dev *usb_0_sd = &devs[0];
	struct serial_dev *uart_0_sd = &devs[1];
	struct device *usb_0_dev;
	struct device *uart_0_dev;

	__ASSERT(hci_uart_dev, "UART device is NULL");

	/* Enable the raw interface, this will in turn open the HCI driver */
	bt_enable_raw(&rx_queue);
	LOG_INF("LTE Gateway BLE started");
	log_uart_pins();

	if (IS_ENABLED(CONFIG_BT_WAIT_NOP)) {
		/* Issue a Command Complete with NOP */
		int i;

		const struct {
			const uint8_t h4;
			const struct bt_hci_evt_hdr hdr;
			const struct bt_hci_evt_cmd_complete cc;
		} __packed cc_evt = {
			.h4 = H4_EVT,
			.hdr = {
				.evt = BT_HCI_EVT_CMD_COMPLETE,
				.len = sizeof(struct bt_hci_evt_cmd_complete),
			},
			.cc = {
				.ncmd = 1,
				.opcode = sys_cpu_to_le16(BT_OP_NOP),
			},
		};

		LOG_INF("Sending HCI Cmd Complete w/ NOP");
		for (i = 0; i < sizeof(cc_evt); i++) {
			uart_poll_out(hci_uart_dev,
				      *(((const uint8_t *)&cc_evt)+i));
		}
	}

	/* Spawn the TX thread and start feeding commands and data to the
	 * controller
	 */
	k_thread_create(&tx_thread_data, tx_thread_stack,
			K_THREAD_STACK_SIZEOF(tx_thread_stack), tx_thread,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);
	k_thread_name_set(&tx_thread_data, "HCI uart TX");

	usb_0_dev = device_get_binding("CDC_ACM_0");
	if (!usb_0_dev) {
		LOG_INF("CDC ACM device not found");
		return;
	}

	uart_0_dev = device_get_binding("UART_0");
	if (!uart_0_dev) {
		LOG_INF("UART 0 init failed");
	}

	usb_0_sd->num = -1;
	usb_0_sd->dev = usb_0_dev;
	usb_0_sd->fifo = &usb_0_tx_fifo;
	usb_0_sd->peer = uart_0_sd;

	uart_0_sd->num = 0;
	uart_0_sd->dev = uart_0_dev;
	uart_0_sd->fifo = &uart_0_tx_fifo;
	uart_0_sd->peer = usb_0_sd;

	k_sem_init(&usb_0_sd->sem, 0, 1);
	k_sem_init(&uart_0_sd->sem, 0, 1);

	uart_irq_callback_user_data_set(usb_0_dev, uart_interrupt_handler,
		usb_0_sd);
	uart_irq_callback_user_data_set(uart_0_dev, uart_interrupt_handler,
		uart_0_sd);

	ret = usb_enable(NULL);
	if (ret != 0) {
		LOG_INF("Failed to enable USB");
		return;
	}

	uart_irq_rx_enable(usb_0_dev);
	uart_irq_rx_enable(uart_0_dev);

	struct k_poll_event events[2] = {
		K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
						K_POLL_MODE_NOTIFY_ONLY,
						&usb_0_sd->sem, 0),
		K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
						K_POLL_MODE_NOTIFY_ONLY,
						&uart_0_sd->sem, 0),
	};

	LOG_INF("Entering loop");

	while (1) {
		struct net_buf *buf;

		buf = net_buf_get(&rx_queue, K_NO_WAIT);
		if (buf) {
			err = h4_send(buf);
			if (err) {
				LOG_ERR("Failed to send");
			}
		}
		ret = k_poll(events, ARRAY_SIZE(events), K_NO_WAIT);
		if (ret != 0) {
			k_sleep(K_MSEC(1));
			continue;
		}

		if (events[0].state == K_POLL_TYPE_SEM_AVAILABLE) {
			events[0].state = K_POLL_STATE_NOT_READY;
			k_sem_take(&usb_0_sd->sem, K_NO_WAIT);
			LOG_DBG("en usb0 tx");
			uart_irq_tx_enable(usb_0_dev);
		} else if (events[1].state == K_POLL_TYPE_SEM_AVAILABLE) {
			events[1].state = K_POLL_STATE_NOT_READY;
			k_sem_take(&uart_0_sd->sem, K_NO_WAIT);
			LOG_DBG("en uart0 tx");
			uart_irq_tx_enable(uart_0_dev);
		}
	}
}
