/*
 * Copyright (c) 2020 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <init.h>
#include <drivers/gpio.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(board_control, CONFIG_LOG_OVERRIDE_LEVEL);

/* Spare Pins
 *
 * | nRF52840 |
 * | P0.06   |
 * | P0.05   |
 * | P0.26   |
 * | P0.27   |
 * | P0.28   |
 * | P0.30   |
 * | P0.03   |
 * | P1.11   |
 */

__packed struct pin_config {
	u8_t pin;
	u8_t val;
};

bool ignore_reset = true;
static void chip_reset(struct device *gpio,
		       struct gpio_callback *cb, u32_t pins)
{
	const u32_t stamp = k_cycle_get_32();

	printk("GPIO reset line asserted, device reset.\n");
	printk("Bye @ cycle32 %u\n", stamp);

	if (!ignore_reset) {
		NVIC_SystemReset();
	}
}

static void reset_pin_wait_low(struct device *port, u32_t pin)
{
	int val;

	/* Wait until the pin is pulled low */
	do {
		val = gpio_pin_get_raw(port, pin);
	} while (val > 0);
}

static int reset_pin_configure(struct device *port, u32_t pin)
{
	int err;

	static struct gpio_callback gpio_ctx;

	__ASSERT_NO_MSG(port != NULL);

	err = gpio_pin_configure(port, pin, GPIO_INPUT | GPIO_PULL_DOWN);
	if (err) {
		return err;
	}

	gpio_init_callback(&gpio_ctx, chip_reset, BIT(pin));

	err = gpio_add_callback(port, &gpio_ctx);
	if (err) {
		return err;
	}

	err = gpio_pin_interrupt_configure(port, pin, GPIO_INT_EDGE_RISING);
	if (err) {
		return err;
	}

	/* Wait until the pin is pulled low before continuing.
	 * This lets the other side ensure that they are ready.
	 */
	LOG_INF("GPIO reset line enabled");

	reset_pin_wait_low(port, pin);

	return 0;
}

static int init(struct device *dev)
{
	/* Make sure to configure the switches before initializing
	 * the GPIO reset pin, so that we are connected to
	 * the nRF9160 before enabling our interrupt.
	 */
	if (IS_ENABLED(CONFIG_BOARD_APRICITY_GATEWAY_NRF52840_RESET)) {
		int rc;
		struct device *port;
		const char *name;

		LOG_INF("Enabling GPIO reset line on pin P%d.%02u..",
			CONFIG_BOARD_APRICITY_GATEWAY_NRF52840_RESET_PORT,
			CONFIG_BOARD_APRICITY_GATEWAY_NRF52840_RESET_PIN);

		if (CONFIG_BOARD_APRICITY_GATEWAY_NRF52840_RESET_PORT == 0) {
			name = DT_LABEL(DT_NODELABEL(gpio0));
		} else {
			name = DT_LABEL(DT_NODELABEL(gpio1));
		}
		port = device_get_binding(name);
		if (!port) {
			LOG_ERR("GPIO device %s not found!", name);
			return -EIO;
		}

		rc = reset_pin_configure(port,
			CONFIG_BOARD_APRICITY_GATEWAY_NRF52840_RESET_PIN);
		if (rc) {
			LOG_ERR("Unable to configure reset pin, err %d", rc);
			return -EIO;
		}
	}

	LOG_INF("Board configured.");

	return 0;
}

SYS_INIT(init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
