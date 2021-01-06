
#include "tk_mqtt_dfu.h"

#include "tk_shutdown.h"

#include <cJSON.h>
#include <cJSON_os.h>

#include <zephyr.h>
#include <logging/log.h>

#include <dfu/flash_img.h>
#include <dfu/mcuboot.h>
#include <power/reboot.h>
#include <sys/base64.h>

#include <string.h>

LOG_MODULE_REGISTER(alt_dfu_stream, CONFIG_LTE_GATEWAY_BLE_LOG_LEVEL);

#define MAX_DATA_LENGTH         2048            // 2048*4/3 = size of base64 string of 2048

#define FILE_SIZE_NAME          "file_size"
#define MAX_CHUNK_NAME          "max_chunk_size"
#define VERSION_NAME            "version"
#define TOTAL_MSQ_COUNT_NAME    "total_msg_count"
#define MSG_ID_NAME             "msg_id"
#define CHUNK_NAME              "chunk_size"
#define DATA_NAME               "data"
#define DEV_ID_NAME             "devID"

static int file_size = 0;
static int max_chunk_size = 0;
static int final_msg = 0;
static int current_msg = 0;

static struct	flash_img_context flash_img;


static cJSON * tk_mqtt_create_request(void);


// static char version[] = "00.00.00" // Not currently using version

// Process a message to start a DFU process
/*
{
    "cmd":CMD_START_DFU,
    "file_size":250000,     // bytes
    "max_chunk_size":2048,  // bytes
    "version":"1.0.0",      // string
    "total_msg_count":245   // Number of messages expected
}
*/
cJSON * tk_mqtt_dfu_start(cJSON *msg){

    int err = flash_img_init(&flash_img);

	if (err != 0) {
		LOG_ERR("flash_img_init error %d", err);
		return NULL;
	}

    current_msg = 0;

    cJSON *fs = cJSON_GetObjectItem(msg,FILE_SIZE_NAME);
    if(fs == NULL)
    {
        LOG_ERR("Missing File Size");
        return NULL;
    }
    file_size = fs->valueint;

    cJSON *mcs = cJSON_GetObjectItem(msg,MAX_CHUNK_NAME);
    if(mcs == NULL)
    {
        LOG_ERR("Missing Chunk Size");
        return NULL;
    }
    max_chunk_size = mcs->valueint;

    cJSON *tmc =  cJSON_GetObjectItem(msg,TOTAL_MSQ_COUNT_NAME);
    if(tmc == NULL)
    {
        LOG_ERR("Missing Total Message Count");
        return NULL;
    }
    final_msg = tmc->valueint - 1;

    // Send request for first message
    return tk_mqtt_create_request();
}


// Process a dfu data message
/*
{
    "cmd":CMD_DFU_DATA,
    "msg_id":145,               // ID of the message
    "chunk_size":2048,          // bytes
    "data":"BASE64ENCODEDDATA"  //String
}
*/
static uint16_t msg_id = 0;
static size_t chunk_size = 0;
static uint8_t data[MAX_DATA_LENGTH];
static size_t olen;
cJSON * tk_mqtt_dfu_data(cJSON *msg){

    int err;

    cJSON *mi = cJSON_GetObjectItem(msg, MSG_ID_NAME);
    msg_id = mi->valueint;

    LOG_INF("msg_id %d, current_id %d", msg_id, current_msg);
    if(msg_id != current_msg)
    {
        return NULL;
    }
    else if ((current_msg-msg_id) == 1)
    {
        //If we received a previous message again, then the central server probably failed to get this
        //request, so send it again.
        return tk_mqtt_create_request();
    }

    cJSON *cs = cJSON_GetObjectItem(msg, CHUNK_NAME);
    chunk_size = cs->valueint;

    // Get data string and decode to uint8_t
    cJSON *d = cJSON_GetObjectItem(msg, DATA_NAME);
    base64_decode(data,chunk_size,&olen,d->valuestring,strlen(d->valuestring));

    LOG_INF("Chunk size:%d, Olen:%d",chunk_size,olen);

    // Write to flash
    err = flash_img_buffered_write(&flash_img,data,olen, false);
    if (err != 0) {
        LOG_ERR("flash_img_buffered_write error %d", err);
        return NULL;
    }

    // If last message
    if(msg_id == final_msg){
		/* Write with 0 length to flush the write operation to flash. */
		err = flash_img_buffered_write(&flash_img, data, 0, true);
		if (err != 0) {
			LOG_ERR("flash_img_buffered_write error %d", err);
			return NULL;
		}
        current_msg = -1;   // -1 is indication of final message received
        tk_mqtt_create_request(); // send back completion message
        k_sleep(K_MSEC(1000));          // to make sure final message response gets out before we reboot
        // Request boot upgrade
		err = boot_request_upgrade(1);
		if (err != 0) {
			LOG_ERR("boot_request_upgrade error %d", err);
			return NULL;
		}

        // Completed upgrade, rebooting
        LOG_ERR("Completed upgrade, rebooting");
        TK_SHUTDOWN_M();

    }else{
        current_msg = current_msg+1;
        return tk_mqtt_create_request();
    }
    return NULL;
}



// Create and send data request message
/*
{
    "devID":"A00004",
    "msg_id":234        // Send a request for message id
}
*/
static cJSON * tk_mqtt_create_request(void){
    cJSON *obj = cJSON_CreateObject();

    if ((NULL == obj)
        || (NULL == cJSON_AddStringToObject(obj, DEV_ID_NAME, "undefined"))
        || (NULL == cJSON_AddNumberToObject(obj, MSG_ID_NAME, current_msg))) {

        cJSON_Delete(obj);
        return NULL;
    }

    return obj;
}
