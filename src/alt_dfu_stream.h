#ifndef ALT_DFU_STREAM_H_
#define ALT_DFU_STREAM_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

void alt_dfu_process(void);

void alt_dfu_receive(const uint8_t *data, size_t len);

bool alt_dfu_is_tx_pending(void);

const uint8_t *alt_dfu_get_pending_tx(size_t *len);

#endif /* ALT_DFU_STREAM_H_ */
