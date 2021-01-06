#ifndef TK_MQTT_DFU_H
#define TK_MQTT_DFU_H

#include "cJSON.h"

#define TK_MQTT_DFU_START_M(message_json)               tk_mqtt_dfu_start(message_json)
#define TK_MQTT_DFU_DATA_M(message_json)                tk_mqtt_dfu_data(message_json)

cJSON * tk_mqtt_dfu_start(cJSON *msg);
cJSON * tk_mqtt_dfu_data(cJSON *msg);


#endif // TK_MQTT_DFU_H
