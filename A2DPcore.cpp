/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/xtensa_api.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2s.h"
#include "freertos/ringbuf.h"
#include "freertos/stream_buffer.h"
#include "A2DP.h"

/* log tag */
#define BT_APP_CORE_TAG    "BT_APP_CORE"

// uint8_t tempStorage[2048];
uint8_t tempStorage[2048];



union bytesToint16 {
    int16_t int16;
    uint8_t bytes[2];
}  bytesToint16;

bool A2DPSink::bt_app_send_msg(bt_app_msg_t *msg)
{
    if (msg == NULL) {
        return false;
    }

    if (xQueueSend(s_bt_app_task_queue, msg, 10 / portTICK_RATE_MS) != pdTRUE) {
        ESP_LOGE(BT_APP_CORE_TAG, "%s xQueue send failed", __func__);
        return false;
    }
    return true;
}

void A2DPSink::bt_app_work_dispatched(bt_app_msg_t *msg)
{
    if (msg->cb) {
        msg->cb(msg->event, msg->param);
    }
}

void A2DPSink::bt_app_task_handler(void *arg)
{
    bt_app_msg_t msg;

    for (;;) {
        /* receive message from work queue and handle it */
        if (pdTRUE == xQueueReceive(s_bt_app_task_queue, &msg, (portTickType)portMAX_DELAY)) {
            ESP_LOGD(BT_APP_CORE_TAG, "%s, signal: 0x%x, event: 0x%x", __func__, msg.sig, msg.event);

            switch (msg.sig) {
            case BT_APP_SIG_WORK_DISPATCH:
                bt_app_work_dispatched(&msg);
                break;
            default:
                ESP_LOGW(BT_APP_CORE_TAG, "%s, unhandled signal: %d", __func__, msg.sig);
                break;
            } /* switch (msg.sig) */

            if (msg.param) {
                free(msg.param);
            }
        }
        vTaskDelay(20/portTICK_PERIOD_MS);
    }
}

void A2DPSink::bt_i2s_task_handler(void *arg)
{
    uint8_t *data1 = NULL;
    uint8_t *data2 = NULL;
    size_t item_size1 = 0;
    size_t item_size2 = 0;
    
    size_t bytes_written = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    // uint8_t storage = malloc(3000);
    // uint16_t head = 0;

    size_t rcvSize;

    for (;;) {
        /* receive data from ringbuffer and write it to I2S DMA transmit buffer */

        // ESP_LOGI("size", "%i", (unsigned int)itemsWaiting);
        // data = (uint8_t *)xRingbufferReceiveUpTo(s_ringbuf_i2s, &item_size, (portTickType)portMAX_DELAY, (size_t ) 3000);
        // data1 = (uint8_t *)xRingbufferReceiveSplit(s_ringbuf_i2s, (void **)&data1, (void **)&data2, &item_size1, &item_size2, (portTickType)1000);


        // if (item_size1 > 0){
        if(xStreamBufferBytesAvailable(i2sStreamBuffer) > 2000){
            rcvSize = xStreamBufferReceive(i2sStreamBuffer, (void*) tempStorage, 2048, (TickType_t) portMAX_DELAY);

            //This is equivalent to (volume/127)^2 * 100
            //This allows us to scale exponentially by multiplying the sample by this value and then dividing by 100
            uint16_t expVolume = ((volume * volume) * 100) / (127 * 127);
            for(int i = 0; i < rcvSize - 1; i+=2){
                bytesToint16.bytes[0] = tempStorage[i];
                bytesToint16.bytes[1] = tempStorage[i + 1];
                bytesToint16.int16 = int16_t(int32_t(bytesToint16.int16 * expVolume)/100);
                tempStorage[i] = bytesToint16.bytes[0];
                tempStorage[i + 1] = bytesToint16.bytes[1];
            }

            i2s_write(i2sPort, tempStorage, rcvSize, &bytes_written, portMAX_DELAY);
            // vRingbufferReturnItem(s_ringbuf_i2s, (void *)data1);
            if(dataCallback != nullptr){
                (*dataCallback)(tempStorage, rcvSize);
            }
        }

        xLastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil( &xLastWakeTime, 10/portTICK_PERIOD_MS);
    }
}

/********************************
 * EXTERNAL FUNCTION DEFINITIONS
 *******************************/

bool A2DPSink::bt_app_work_dispatch(bt_app_cb_t p_cback, uint16_t event, void *p_params, int param_len, bt_app_copy_cb_t p_copy_cback)
{
    ESP_LOGD(BT_APP_CORE_TAG, "%s event: 0x%x, param len: %d", __func__, event, param_len);

    bt_app_msg_t msg;
    memset(&msg, 0, sizeof(bt_app_msg_t));

    msg.sig = BT_APP_SIG_WORK_DISPATCH;
    msg.event = event;
    msg.cb = p_cback;

    if (param_len == 0) {
        return bt_app_send_msg(&msg);
    } else if (p_params && param_len > 0) {
        if ((msg.param = malloc(param_len)) != NULL) {
            memcpy(msg.param, p_params, param_len);
            /* check if caller has provided a copy callback to do the deep copy */
            if (p_copy_cback) {
                p_copy_cback(msg.param, p_params, param_len);
            }
            return bt_app_send_msg(&msg);
        }
    }

    return false;
}

void A2DPSink::bt_app_task_start_up(void)
{
    s_bt_app_task_queue = xQueueCreate(10, sizeof(bt_app_msg_t));
    xTaskCreatePinnedToCore(bt_app_task_handler, "BtAppTask", 3072, NULL, 4, &s_bt_app_task_handle, taskCore);
}

void A2DPSink::bt_app_task_shut_down(void)
{
    if (s_bt_app_task_handle) {
        vTaskDelete(s_bt_app_task_handle);
        s_bt_app_task_handle = NULL;
    }
    if (s_bt_app_task_queue) {
        vQueueDelete(s_bt_app_task_queue);
        s_bt_app_task_queue = NULL;
    }
}

void A2DPSink::bt_i2s_task_start_up(void)
{
    i2sStreamBuffer = xStreamBufferCreate(8000, 1);

    xTaskCreatePinnedToCore(bt_i2s_task_handler, "BtI2STask", 3072, NULL, configMAX_PRIORITIES - 3, &s_bt_i2s_task_handle, taskCore);
}

void A2DPSink::bt_i2s_task_shut_down(void)
{
    if (s_bt_i2s_task_handle) {
        vTaskDelete(s_bt_i2s_task_handle);
        s_bt_i2s_task_handle = NULL;
    }
    if(i2sStreamBuffer){
        vStreamBufferDelete(i2sStreamBuffer);
        i2sStreamBuffer = NULL;
    }
}