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
#include "A2DP.h"

/* log tag */
#define BT_APP_CORE_TAG    "BT_APP_CORE"

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
    uint8_t *data = NULL;
    size_t item_size = 0;
    size_t bytes_written = 0;

    for (;;) {
        /* receive data from ringbuffer and write it to I2S DMA transmit buffer */

        data = (uint8_t *)xRingbufferReceive(s_ringbuf_i2s, &item_size, (portTickType)portMAX_DELAY);

        if (item_size != 0){
            
            //This is equivalent to (volume/127)^2 * 100
            //This allows us to scale exponentially by multiplying the sample by this value and then dividing by 100
            uint16_t expVolume = ((volume * volume) * 100) / (127 * 127);
            for(int i = 0; i < item_size - 1; i+=2){
                bytesToint16.bytes[0] = data[i];
                bytesToint16.bytes[1] = data[i + 1];
                bytesToint16.int16 = int16_t(int32_t(bytesToint16.int16 * expVolume)/100);
                data[i] = bytesToint16.bytes[0];
                data[i + 1] = bytesToint16.bytes[1];
            }

            i2s_write(i2sPort, data, item_size, &bytes_written, portMAX_DELAY);
            vRingbufferReturnItem(s_ringbuf_i2s, (void *)data);
        }
        vTaskDelay(5/portTICK_PERIOD_MS);
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
    xTaskCreatePinnedToCore(bt_app_task_handler, "BtAppTask", 3072, NULL, 10, &s_bt_app_task_handle, taskCore);
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
    if ((s_ringbuf_i2s = xRingbufferCreate(8 * 1024, RINGBUF_TYPE_BYTEBUF)) == NULL) {
        return;
    }
    xTaskCreatePinnedToCore(bt_i2s_task_handler, "BtI2STask", 1024, NULL, configMAX_PRIORITIES - 3, &s_bt_i2s_task_handle, taskCore);
}

void A2DPSink::bt_i2s_task_shut_down(void)
{
    if (s_bt_i2s_task_handle) {
        vTaskDelete(s_bt_i2s_task_handle);
        s_bt_i2s_task_handle = NULL;
    }
    if (s_ringbuf_i2s) {
        vRingbufferDelete(s_ringbuf_i2s);
        s_ringbuf_i2s = NULL;
    }
}

size_t A2DPSink::write_ringbuf(const uint8_t *data, size_t size)
{
    BaseType_t done = xRingbufferSend(s_ringbuf_i2s, (void *)data, size, (portTickType)portMAX_DELAY);

    return done ? size : 0;
}
