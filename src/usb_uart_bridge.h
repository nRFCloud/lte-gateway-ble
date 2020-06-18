
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
	u8_t buffer[UART_BUF_SIZE];
	u16_t len;
};

static struct serial_dev {
	struct device *dev;
	void *peer;
	struct k_fifo *fifo;
	struct k_sem sem;
	struct uart_data *rx;
} devs[2];


void uart_interrupt_handler(void *user_data);
void power_thread(void);