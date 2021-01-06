
#include "alt_dfu_stream.h"

#include "tk_mqtt_dfu.h"

#include "cJSON.h"

#include <kernel.h>

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* Copied from dfu_target_peripheral.h */
#define DFU_PACKET_MAGIC_LEN (8u)
#define DFU_MAGIC_STRING "XoPU24Tk"

#define DFU_PACKET_HEADER_LEN (DFU_PACKET_MAGIC_LEN + 6u)
#define DFU_HEADER_FMT (DFU_MAGIC_STRING " %04d %s\n")

#define DFU_MAX_CHUNK_LEN (128u)

#define DFU_JSON_MAX_B64_DATA_LEN (4u * (DFU_MAX_CHUNK_LEN / 3u) + 3u)
#define DFU_JSON_MAX_LEN (DFU_JSON_MAX_B64_DATA_LEN + 100u)

#define DFU_PACKET_MAX_BUFFER_LEN \
	(DFU_PACKET_HEADER_LEN + DFU_JSON_MAX_LEN + 2u)

#define TX_PACKET_MAX_BUFFER_LEN (200u)

#define CMD_START_DFU   (7u)
#define CMD_DFU_DATA    (9u)


/* Local buffers */
K_MUTEX_DEFINE(rx_buffer_mutex);
static uint8_t rx_buffer[DFU_PACKET_MAX_BUFFER_LEN];
static volatile size_t rx_buffer_len = 0;

static char resp_str[(TX_PACKET_MAX_BUFFER_LEN - DFU_PACKET_HEADER_LEN) + 1];
static uint8_t tx_buffer[TX_PACKET_MAX_BUFFER_LEN];
static size_t tx_buffer_len = 0;


static inline void trim_rx_buffer(size_t idx)
{
	k_mutex_lock(&rx_buffer_mutex, K_FOREVER);

	if (idx >= rx_buffer_len) {
		rx_buffer_len = 0u;
	} else if (idx > 0) {
		rx_buffer_len = rx_buffer_len - idx;
		memmove(rx_buffer, rx_buffer+idx, rx_buffer_len);
	}

	k_mutex_unlock(&rx_buffer_mutex);
}

static void build_response_pkt(cJSON * response)
{
	int olen = 0;

	if ((NULL != response) &&
		cJSON_PrintPreallocated(response, resp_str, sizeof(resp_str), false)) {
		size_t json_len = strlen(resp_str);
		olen = snprintf((char *) tx_buffer, sizeof(tx_buffer),
						DFU_HEADER_FMT, json_len, resp_str);
	} else {
		int olen = snprintf((char *) tx_buffer, sizeof(tx_buffer),
							DFU_HEADER_FMT, 2, "{}");
	}

	if (olen < sizeof(tx_buffer)) {
		tx_buffer_len = olen;
	} else {
		tx_buffer_len = 0;
	}
}

static void process_rx_pkt(const char *json_str)
{
	cJSON *obj = cJSON_Parse(json_str);
	cJSON *response = NULL;

	if (NULL != obj) {
		cJSON * cmd_obj = cJSON_GetObjectItem(obj, "cmd");
		if ((NULL != cmd_obj) && cJSON_IsNumber(cmd_obj)) {
			switch(cmd_obj->valueint)
			{
			case CMD_START_DFU:
				response = TK_MQTT_DFU_START_M(obj);
				break;
			case CMD_DFU_DATA:
				response = TK_MQTT_DFU_DATA_M(obj);
				break;
			default:
				break;
			}
		}
	}

	build_response_pkt(response);
	cJSON_Delete(response);
	cJSON_Delete(obj);
}

static void process_rx_buffer(void)
{
	do
	{
		if (rx_buffer_len < DFU_PACKET_HEADER_LEN) {
			/* Not enough data to process*/
			break;
		}

		/* Trim off any garbage in the buffer */
		size_t trim_idx = 0;
		for (; trim_idx < (rx_buffer_len - DFU_PACKET_MAGIC_LEN); ++trim_idx) {
			if (DFU_MAGIC_STRING[0] == rx_buffer[trim_idx]) {
				break;
			}
		}
		if (trim_idx > 0) {
			trim_rx_buffer(trim_idx);
			continue;
		}

		if (0 != memcmp(rx_buffer, DFU_MAGIC_STRING, DFU_PACKET_MAGIC_LEN)) {
			/* Magic string does not match, remove the garbage */
			trim_rx_buffer(1);
			continue;
		}

		/* Grab the length field */
		size_t json_len = 0u;
		{
			char len_str[5];
			memcpy(len_str, &rx_buffer[DFU_PACKET_MAGIC_LEN + 1], 4);
			len_str[4] = '\0';

			long int raw_len = strtol(len_str, NULL, 10);
			if ((0 == raw_len) || (raw_len > DFU_JSON_MAX_LEN)) {
				/* Invalid length field, thrown the mysteriously matching magic
				   string */
				trim_rx_buffer(DFU_PACKET_MAGIC_LEN);
				continue;
			}
			json_len = (size_t) raw_len;
		}

		size_t pkt_len = json_len + DFU_PACKET_HEADER_LEN;
		if (rx_buffer_len < pkt_len) {
			/* Not enough bytes to extract the JSON data */
			break;
		}

		const char *json_str = (const char *) &rx_buffer[DFU_PACKET_HEADER_LEN];
		rx_buffer[pkt_len] = '\0';

		/* Complete packet has been received, process it. We will break
		   from the loop here so that a response message can be sent back */
		process_rx_pkt(json_str);
		trim_rx_buffer(pkt_len);
	} while(0);
}

void alt_dfu_process(void)
{
	if (tx_buffer_len == 0) {
		/* Only process data if there isn't an outgoing packet */
		process_rx_buffer();
	}
}

void alt_dfu_receive(const uint8_t *data, size_t len)
{
	k_mutex_lock(&rx_buffer_mutex, K_FOREVER);

	size_t to_copy = len;
	size_t bytes_free = sizeof(rx_buffer) - rx_buffer_len;
	if (bytes_free < len)
		to_copy = bytes_free;

	memcpy(&rx_buffer[rx_buffer_len], data, to_copy);
	rx_buffer_len += to_copy;

	k_mutex_unlock(&rx_buffer_mutex);
}

bool alt_dfu_is_tx_pending(void)
{
	return (tx_buffer_len > 0);
}

const uint8_t *alt_dfu_get_pending_tx(size_t *len)
{
	if (tx_buffer_len > 0) {
		*len = tx_buffer_len;
		tx_buffer_len = 0;
		return tx_buffer;
	} else {
		*len = 0;
		return NULL;
	}
}
