#ifndef A2DP_H
#define A2DP_H
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"

#include "freertos/xtensa_api.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "esp_log.h"
// #include "bt_app_core.h"
#include "driver/i2s.h"
#include "freertos/ringbuf.h"

#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"

#include "sys/lock.h"

/* log tags */
#define BT_AV_TAG       "BT_AV"
#define BT_RC_TG_TAG    "RC_TG"
#define BT_RC_CT_TAG    "RC_CT"


/* event for stack up */
enum {
    START_UP_EVENT = 0,
};

/* signal for `bt_app_work_dispatch` */
#define BT_APP_SIG_WORK_DISPATCH    (0x01)

/**
 * @brief  handler for the dispatched work
 *
 * @param [in] event  event id
 * @param [in] param  handler parameter
 */
typedef void (* bt_app_cb_t) (uint16_t event, void *param);

/* message to be sent */
typedef struct {
    uint16_t       sig;      /*!< signal to bt_app_task */
    uint16_t       event;    /*!< message event id */
    bt_app_cb_t    cb;       /*!< context switch callback */
    void           *param;   /*!< parameter area needs to be last */
} bt_app_msg_t;

/**
 * @brief  parameter deep-copy function to be customized
 *
 * @param [out] p_dest  pointer to destination data
 * @param [in]  p_src   pointer to source data
 * @param [in]  len     data length in byte
 */
typedef void (* bt_app_copy_cb_t) (void *p_dest, void *p_src, int len);

class A2DPSink{
    public:
        /*Main functions*/
        A2DPSink();
        void start();
        void setExternalAudioDataCallback(void (*callback)(const uint8_t*, uint32_t));
        void setOnConnectCallback(void (*callback)());
        void setOnDisconnectCallback(void (*callback)());
        void setI2SPort(i2s_port_t port);
        void setI2SPins(uint8_t bclk, uint8_t wclk, uint8_t data);
        void setBTName(char *name);
        static bool getDeviceConnected();
        uint8_t getVolume();
        static uint16_t getSampleRate();

    private:

        static bool connected;
        static void (*dataCallback)(const uint8_t*, uint32_t);
        static void (*connectedCallback)();
        static void (*disconnectedCallback)();

        /*Main Functions*/
        static void bluetoothGAPCallback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
        static void bluetoothAudioEventHandler(uint16_t event, void *p_param);

        static i2s_config_t i2sConfig;
        static i2s_pin_config_t i2sPins;
        static i2s_port_t i2sPort;
        static esp_bt_pin_code_t pinCode;
        static char *btName;

        /*Functions from bt_app_av.c from the ESP A2DP example*/
        /* allocate new meta buffer */
        static void bt_app_alloc_meta_buffer(esp_avrc_ct_cb_param_t *param);
        /* handler for new track is loaded */
        static void bt_av_new_track(void);
        /* handler for track status change */
        static void bt_av_playback_changed(void);
        /* handler for track playing position change */
        static void bt_av_play_pos_changed(void);
        /* notification event handler */
        static void bt_av_notify_evt_handler(uint8_t event_id, esp_avrc_rn_param_t *event_parameter);
        /* installation for i2s */
        static void bt_i2s_driver_install(void);
        /* uninstallation for i2s */
        static void bt_i2s_driver_uninstall(void);
        /* set volume by remote controller */
        static void volume_set_by_controller(uint8_t v);
        /* set volume by local host */
        static void volume_set_by_local_host(uint8_t v);
        /* a2dp event handler */
        static void bt_av_hdl_a2d_evt(uint16_t event, void *p_param);
        /* avrc controller event handler */
        static void bt_av_hdl_avrc_ct_evt(uint16_t event, void *p_param);
        /* avrc target event handler */
        static void bt_av_hdl_avrc_tg_evt(uint16_t event, void *p_param);
        
        static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);

        static void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len);

        static void bt_app_rc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param);

        static void bt_app_rc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param);

        static uint16_t sampleRate;

        /* audio stream datapath state */
        esp_a2d_audio_state_t s_audio_state = ESP_A2D_AUDIO_STATE_STOPPED;
        // /* connection state in string */
        // const char *s_a2d_conn_state_str[4] = {"Disconnected", "Connecting", "Connected", "Disconnecting"};
        // /* audio stream datapath state in string */
        // const char *s_a2d_audio_state_str[3] = {"Suspended", "Stopped", "Started"};
        /* AVRC target notification capability bit mask */
        static esp_avrc_rn_evt_cap_mask_t s_avrc_peer_rn_cap;
        
        /*A lock to prevent data races when setting local and remote volume*/
        static _lock_t volume_lock;
        /* local volume value */
        static uint8_t volume;
        /* notify volume change or not */
        static bool volume_notify;

        /*Functions from bt_app_core.c from the ESP A2DP example*/
        /* handler for application task */
        static void bt_app_task_handler(void *arg);
        /* handler for I2S task */
        static void bt_i2s_task_handler(void *arg);
        /* message sender */
        static bool bt_app_send_msg(bt_app_msg_t *msg);
        /* handle dispatched messages */
        static void bt_app_work_dispatched(bt_app_msg_t *msg);

        static bool bt_app_work_dispatch(bt_app_cb_t p_cback, uint16_t event, void *p_params, int param_len, bt_app_copy_cb_t p_copy_cback);

        static void bt_app_task_start_up(void);

        static void bt_app_task_shut_down(void);

        static void bt_i2s_task_start_up(void);

        static void bt_i2s_task_shut_down(void);

        static size_t write_ringbuf(const uint8_t *data, size_t size);

        static xQueueHandle s_bt_app_task_queue;  /* handle of work queue */
        static xTaskHandle s_bt_app_task_handle;  /* handle of application task  */
        static xTaskHandle s_bt_i2s_task_handle;  /* handle of I2S task */
        static StreamBufferHandle_t i2sStreamBuffer;
        // static RingbufHandle_t s_ringbuf_i2s;     /* handle of ringbuffer for I2S */

        static bool taskCore;
};
#endif