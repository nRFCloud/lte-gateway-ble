#ifndef _USB_UART_BRIDGE_H_
#define _USB_UART_BRIDGE_H_

#define POWER_THREAD_STACKSIZE		CONFIG_IDLE_STACK_SIZE
#define POWER_THREAD_PRIORITY		K_LOWEST_APPLICATION_THREAD_PRIO

/* Heap block space is always one of 2^(2n) for n from 3 to 7.
 * (see reference/kernel/memory/heap.html for more info)
 * Here, we target 64-byte blocks. Since we want to fit one struct uart_data
 * into each block, the block layout becomes:
 * 16 bytes: reserved by the Zephyr heap block descriptor (not in struct)
 *  4 bytes: reserved by the Zephyr FIFO (in struct)
 * 40 bytes: UART data buffer (in struct)
 *  4 bytes: length field (in struct, padded for alignment)
 */
#define UART_BUF_SIZE 40

static K_FIFO_DEFINE(usb_0_tx_fifo);
static K_FIFO_DEFINE(uart_0_tx_fifo);

struct uart_data {
	void *fifo_reserved;
	uint8_t buffer[UART_BUF_SIZE];
	uint16_t len;
};

static struct serial_dev {
	struct k_timer timer;
	const struct device *dev;
	void *peer;
	struct k_fifo *fifo;
	struct k_sem sem;
	struct uart_data *rx;
	int num;
} devs[2];

void uart_bridge_init(struct serial_dev *sd0, struct serial_dev *sd1);
void uart_interrupt_handler(const struct device *dev, void *user_data);
void power_thread(void);

#endif
