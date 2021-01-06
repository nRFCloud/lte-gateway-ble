/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/uart.h>
#include <nrfx.h>
#include <string.h>
#include <stdio.h>
#include <hal/nrf_power.h>
#include <power/reboot.h>
#include <usb/usb_device.h>
#include <logging/log.h>
#include <logging/log_ctrl.h>

#include "usb_uart_bridge.h"

#define UART_IRQ_TX_EMPTY_LOOP_COUNT 1000000
/* #define LOG_CONTENTS */

LOG_MODULE_REGISTER(usb_uart_bridge, CONFIG_LTE_GATEWAY_BLE_LOG_LEVEL);

/* Overriding weak function to set iSerial runtime. */
uint8_t *usb_update_sn_string_descriptor(void)
{
	static uint8_t buf[] = "NRFBLEGW_12PLACEHLDRS";

	snprintk(&buf[9], 13, "%04X%08X",
		(uint32_t)(NRF_FICR->DEVICEADDR[1] & 0x0000FFFF)|0x0000C000,
		(uint32_t)NRF_FICR->DEVICEADDR[0]);

	return (uint8_t *)&buf;
}



/* Frees data for incoming transmission on device blocked by full heap. */
static int oom_free(struct serial_dev *sd)
{
	struct serial_dev *peer_sd = (struct serial_dev *)sd->peer;
	struct uart_data *buf;

	/* First, try to free from FIFO of peer device (blocked stream) */
	buf = k_fifo_get(peer_sd->fifo, K_NO_WAIT);
	if (buf) {
		k_free(buf);
		return 0;
	}

	/* Then, try FIFO of the receiving device (reverse of blocked stream) */
	buf = k_fifo_get(sd->fifo, K_NO_WAIT);
	if (buf) {
		k_free(buf);
		return 0;
	}

	/* Finally, try all of them */
	for (int i = 0; i < ARRAY_SIZE(devs); i++) {
		buf = k_fifo_get(sd->fifo, K_NO_WAIT);
		if (buf) {
			k_free(buf);
			return 0;
		}
	}

	return -1; /* Was not able to free any heap memory */
}

void uart_timer_handler(struct k_timer *timer_id)
{
	void *user_data = k_timer_user_data_get(timer_id);
	struct serial_dev *sd = user_data;
	struct serial_dev *peer_sd = (struct serial_dev *)sd->peer;
	struct uart_data *rx;

	rx = sd->rx;
	sd->rx = NULL;
	if (rx) {
		k_fifo_put(peer_sd->fifo, rx);
		k_sem_give(&peer_sd->sem);
	} else  {
		LOG_DBG("timer ignored");
	}

}

void uart_bridge_init(struct serial_dev *sd0, struct serial_dev *sd1)
{
	k_timer_init(&sd0->timer, uart_timer_handler, NULL);
	k_timer_init(&sd1->timer, uart_timer_handler, NULL);
}

void uart_interrupt_handler(const struct device *dev, void *user_data)
{
	struct serial_dev *sd = user_data;
	struct serial_dev *peer_sd = (struct serial_dev *)sd->peer;
	struct uart_data *rx;
#if defined(LOG_CONTENTS)
	static char rxlabel[32];
	static char txlabel[32];
#endif

	uart_irq_update(dev);

	while (uart_irq_rx_ready(dev)) {
		int data_length;

		while (!sd->rx) {
			sd->rx = k_malloc(sizeof(*sd->rx));
			if (sd->rx) {
				sd->rx->len = 0;
			} else {
				int err = oom_free(sd);

				if (err) {
					LOG_ERR("Could not free memory."
						" Rebooting.");
					sys_reboot(SYS_REBOOT_COLD);
				}
			}
		}

		data_length = uart_fifo_read(dev, &sd->rx->buffer[sd->rx->len],
					   UART_BUF_SIZE - sd->rx->len);
		sd->rx->len += data_length;

		if (sd->rx->len > 0) {
			if ((sd->rx->len == UART_BUF_SIZE) ||
			   (sd->rx->buffer[sd->rx->len - 1] == '\n') ||
			   (sd->rx->buffer[sd->rx->len - 1] == '\r') ||
			   (sd->rx->buffer[sd->rx->len - 1] == '\0')) {
#if defined(LOG_CONTENTS)
				sprintf(rxlabel, "uart_fifo_rx %d", sd->num);
				LOG_HEXDUMP_DBG(sd->rx->buffer,
					sd->rx->len, rxlabel);
#else
				LOG_DBG("rx%d-%d RCVD", sd->num, sd->rx->len);
#endif
				k_timer_stop(&sd->timer);
				rx = sd->rx;
				sd->rx = NULL;
				k_fifo_put(peer_sd->fifo, rx);
				k_sem_give(&peer_sd->sem);
			} else if (!k_timer_remaining_ticks(&sd->timer)) {
				k_timer_user_data_set(&sd->timer, user_data);
				k_timer_start(&sd->timer, K_MSEC(50),
					      K_NO_WAIT);
			}
		}
	}

	if (uart_irq_tx_ready(dev)) {
		struct uart_data *buf = k_fifo_get(sd->fifo, K_NO_WAIT);
		uint16_t written = 0;

		/* Nothing in the FIFO, nothing to send */
		if (!buf) {
			uart_irq_tx_disable(dev);
			return;
		}

#if defined(LOG_CONTENTS)
		if (buf->len) {
			sprintf(txlabel, "uart_fifo_tx %d", sd->num);
			LOG_HEXDUMP_DBG(buf->buffer, buf->len,
				txlabel);
		}
#else
		if (buf->len) {
			LOG_DBG("tx%d-%d SENT", sd->num, buf->len);
		}
#endif

		while (buf->len > written) {
			if (uart_irq_tx_ready(dev)) {
				written += uart_fifo_fill(dev,
							  &buf->buffer[written],
							  buf->len - written);
			}
		}

		int i = 0;

		while (!uart_irq_tx_complete(dev)) {
			/* Wait for the last byte to get
			 * shifted out of the module
			 */
			if (i++ > UART_IRQ_TX_EMPTY_LOOP_COUNT) {
				LOG_ERR("timeout on tx complete");
				break;
			}
		}

		if (k_fifo_is_empty(sd->fifo)) {
			uart_irq_tx_disable(dev);
		}

		k_free(buf);
	}
}

void power_thread(void)
{
	while (1) {
		if (!nrf_power_usbregstatus_vbusdet_get(NRF_POWER)) {
			nrf_power_system_off(NRF_POWER);
		}
		k_sleep(K_MSEC(100));
	}
}

K_THREAD_DEFINE(power_thread_id, POWER_THREAD_STACKSIZE, power_thread,
		NULL, NULL, NULL, POWER_THREAD_PRIORITY, 0, 0);
