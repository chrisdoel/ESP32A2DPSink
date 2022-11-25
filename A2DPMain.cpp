#include "A2DP.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"

#include "esp_bt.h"
// #include "bt_app_core.h"
// #include "bt_app_av.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "driver/i2s.h"

i2s_config_t A2DPSink::i2sConfig;
i2s_pin_config_t A2DPSink::i2sPins;
i2s_port_t A2DPSink::i2sPort;
esp_bt_pin_code_t A2DPSink::pinCode;
char *A2DPSink::btName;

xQueueHandle A2DPSink::s_bt_app_task_queue;
xTaskHandle A2DPSink::s_bt_app_task_handle;
xTaskHandle A2DPSink::s_bt_i2s_task_handle;
// RingbufHandle_t A2DPSink::s_ringbuf_i2s;
bool A2DPSink::taskCore;

uint8_t A2DPSink::volume;
bool A2DPSink::volume_notify;
uint16_t A2DPSink::sampleRate;

void (*A2DPSink::dataCallback)(const uint8_t*, uint32_t);
void (*A2DPSink::connectedCallback)();
void (*A2DPSink::disconnectedCallback)();

A2DPSink::A2DPSink(){
    //Initialise handles
    s_bt_app_task_queue = NULL;
    s_bt_app_task_handle = NULL;
    s_bt_i2s_task_handle = NULL;
    // s_ringbuf_i2s = NULL;

    //Initialise default i2s configuration
    i2sConfig = { //Setup i2s peripheral 1 to send bt audio packets to dac
        .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = 44100, // corrected by info from bluetooth
        .bits_per_sample = (i2s_bits_per_sample_t) 16, /* the DAC module will only take the 8bits from MSB */
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_MSB,//I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = 0, // default interrupt priority
        .dma_buf_count = 4,
        .dma_buf_len = 256,
        .use_apll = false,
        .tx_desc_auto_clear = true
    };

    //Initialise default i2s pins
    i2sPins = {
      .bck_io_num = 18,
      .ws_io_num = 21,
      .data_out_num = 19,
      .data_in_num = I2S_PIN_NO_CHANGE
    };

    //Initialise default i2s port
    i2sPort = (i2s_port_t) 1;

    //Initialise default pincode
    pinCode[0] = '1';
    pinCode[1] = '2';
    pinCode[2] = '3';
    pinCode[3] = '4';

    //Initialise volume information
    volume = 0;
    volume_notify = false;

    //Initialise the default core the tasks will run on
    taskCore = 0;

    //Initialise the default bluetooth name
    btName = "ESP32 Audio";

    //Initialise the default callback function pointers
    dataCallback = nullptr;
    connectedCallback = nullptr;
    disconnectedCallback = nullptr;
}

void A2DPSink::start(){
    /* initialize NVS — it is used to store PHY calibration data */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((err = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(err));
        return;
    }
    if ((err = esp_bt_controller_enable(ESP_BT_MODE_BTDM)) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(err));
        return;
    }
    if ((err = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(err));
        return;
    }
    if ((err = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(err));
        return;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    /* set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /* set default parameters for Legacy Pairing (use fixed pin code 1234) */
    esp_bt_pin_type_t pinType = ESP_BT_PIN_TYPE_FIXED;
    esp_bt_gap_set_pin(pinType, 4, pinCode);

    bt_app_task_start_up();
    /* bluetooth device name, connection mode and profile set up */
    bt_app_work_dispatch(bluetoothAudioEventHandler, START_UP_EVENT, NULL, 0, NULL);
}

void A2DPSink::setExternalAudioDataCallback(void (*callback)(const uint8_t*, uint32_t)){
    dataCallback = callback;
}

void A2DPSink::setOnConnectCallback(void (*callback)()){
    connectedCallback = callback;
}

void A2DPSink::setOnDisconnectCallback(void (*callback)()){
    disconnectedCallback = callback;
}

void A2DPSink::setI2SPort(i2s_port_t port){
    i2sPort = port;
}

void A2DPSink::setI2SPins(uint8_t bclk, uint8_t wclk, uint8_t data){
    i2sPins = {
      .bck_io_num = bclk,
      .ws_io_num = wclk,
      .data_out_num = data,
      .data_in_num = I2S_PIN_NO_CHANGE
    };
}

void A2DPSink::setBTName(char *name){
    btName = name;
}

bool A2DPSink::getDeviceConnected(){
    return connected;
}

uint8_t A2DPSink::getVolume(){
    return volume;
}

uint16_t A2DPSink::getSampleRate(){
    return sampleRate;
}


void A2DPSink::bluetoothGAPCallback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param){
    switch (event) {
    /* when authentication completed, this event comes */
    case ESP_BT_GAP_AUTH_CMPL_EVT: {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(BT_AV_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(BT_AV_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(BT_AV_TAG, "authentication failed, status: %d", param->auth_cmpl.stat);
        }
        break;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    /* when Security Simple Pairing user confirmation requested, this event comes */
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    /* when Security Simple Pairing passkey notified, this event comes */
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey: %d", param->key_notif.passkey);
        break;
    /* when Security Simple Pairing passkey requested, this event comes */
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif

    /* when GAP mode changed, this event comes */
    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode: %d", param->mode_chg.mode);
        break;
    /* others */
    default: {
        ESP_LOGI(BT_AV_TAG, "event: %d", event);
        break;
    }
    }
}

void A2DPSink::bluetoothAudioEventHandler(uint16_t event, void *p_param){
    ESP_LOGD(BT_AV_TAG, "%s event: %d", __func__, event);

    switch (event) {
    /* when do the stack up, this event comes */
    case START_UP_EVENT: {
        esp_bt_dev_set_device_name(btName);
        esp_bt_gap_register_callback(bluetoothGAPCallback);

        assert(esp_avrc_ct_init() == ESP_OK);
        esp_avrc_ct_register_callback(bt_app_rc_ct_cb);
        assert(esp_avrc_tg_init() == ESP_OK);
        esp_avrc_tg_register_callback(bt_app_rc_tg_cb);

        esp_avrc_rn_evt_cap_mask_t evt_set = {0};
        esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set, ESP_AVRC_RN_VOLUME_CHANGE);
        assert(esp_avrc_tg_set_rn_evt_cap(&evt_set) == ESP_OK);

        assert(esp_a2d_sink_init() == ESP_OK);
        esp_a2d_register_callback(&bt_app_a2d_cb);
        esp_a2d_sink_register_data_callback(bt_app_a2d_data_cb);

        /* set discoverable and connectable mode, wait to be connected */
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        break;
    }
    /* others */
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
}