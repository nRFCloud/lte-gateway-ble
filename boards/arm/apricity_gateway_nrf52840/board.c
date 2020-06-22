/*
 * Copyright (c) 2018 Nordic Semiconductor ASA.
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

static void chip_reset(struct device *gpio,
		       struct gpio_callback *cb, u32_t pins)
{
	const u32_t stamp = k_cycle_get_32();

	printk("GPIO reset line asserted, device reset.\n");
	printk("Bye @ cycle32 %u\n", stamp);

	NVIC_SystemReset();
}

static void reset_pin_wait_low(struct device *port, u32_t pin)
{
	int val;

	/* Wait until the pin is pulled low */
	do {
		val = gpio_pin_get_raw(port, pin);
	} while (val > 0);
}

static int reset_pin_configure(struct device *p0, struct device *p1)
{
	int err;
	u32_t pin = 0;
	struct device *port = NULL;

	static struct gpio_callback gpio_ctx;

        port = p1;
        pin = 2;
	
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
	LOG_INF("GPIO reset line enabled on pin %s.%02u, holding..",
		port == p0 ? "P0" : "P1", pin);

	reset_pin_wait_low(port, pin);

	return 0;
}

static int init(struct device *dev)
{
	int rc;
	struct device *p0;
	struct device *p1;

	p0 = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
	if (!p0) {
		LOG_ERR("GPIO device " DT_LABEL(DT_NODELABEL(gpio0))
			" not found!");
		return -EIO;
	}

	p1 = device_get_binding(DT_LABEL(DT_NODELABEL(gpio1)));
	if (!p1) {
		LOG_ERR("GPIO device " DT_LABEL(DT_NODELABEL(gpio1))
			" not found!");
		return -EIO;
	}

	/* Make sure to configure the switches before initializing
	 * the GPIO reset pin, so that we are connected to
	 * the nRF9160 before enabling our interrupt.
	 */
	//if (IS_ENABLED(CONFIG_BOARD_PCA20035_NRF52840_RESET)) {
		rc = reset_pin_configure(p0, p1);
		if (rc) {
			LOG_ERR("Unable to configure reset pin, err %d", rc);
			return -EIO;
		}
	//}

	LOG_INF("Board configured.");

	return 0;
}

SYS_INIT(init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
