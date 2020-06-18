
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
#include <hal/nrf_power.h>
#include <power/reboot.h>
#include <usb/usb_device.h>


#include "usb_uart_bridge.h"

/* Overriding weak function to set iSerial runtime. */
u8_t *usb_update_sn_string_descriptor(void)
{
	static u8_t buf[] = "PCA20035_12PLACEHLDRS";

	snprintk(&buf[9], 13, "%04X%08X",
		(uint32_t)(NRF_FICR->DEVICEADDR[1] & 0x0000FFFF)|0x0000C000,
		(uint32_t)NRF_FICR->DEVICEADDR[0]);

	return (u8_t *)&buf;
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

void uart_interrupt_handler(void *user_data)
{
	struct serial_dev *sd = user_data;
	struct device *dev = sd->dev;
	struct serial_dev *peer_sd = (struct serial_dev *)sd->peer;

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
					printk("Could not free memory. Rebooting.\n");
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
				k_fifo_put(peer_sd->fifo, sd->rx);
				k_sem_give(&peer_sd->sem);

				sd->rx = NULL;
			}
		}
	}

	if (uart_irq_tx_ready(dev)) {
		struct uart_data *buf = k_fifo_get(sd->fifo, K_NO_WAIT);
		u16_t written = 0;

		/* Nothing in the FIFO, nothing to send */
		if (!buf) {
			uart_irq_tx_disable(dev);
			return;
		}

		while (buf->len > written) {
			written += uart_fifo_fill(dev,
						  &buf->buffer[written],
						  buf->len - written);
		}

		while (!uart_irq_tx_complete(dev)) {
			/* Wait for the last byte to get
			 * shifted out of the module
			 */
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
		k_sleep(100);
	}
}

K_THREAD_DEFINE(power_thread_id, POWER_THREAD_STACKSIZE, power_thread,
		NULL, NULL, NULL, POWER_THREAD_PRIORITY, 0, K_NO_WAIT);
