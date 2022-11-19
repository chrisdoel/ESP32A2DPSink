#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "A2DP.h"

A2DPSink btAudio;

uint32_t audioBytes = 0;

static const char* TAG = "test";

void dataFunc(const uint8_t *data, uint32_t length){
    audioBytes += length;
}

void connectedFunc(){
    ESP_LOGI("ConnectedCB", "Connected");
}

void disconnectedFunc(){
    ESP_LOGI("DisconnectedCB", "Disconnected");
}


extern "C" void app_main(void)
{
    btAudio.setExternalAudioDataCallback(dataFunc);
    btAudio.setOnConnectCallback(connectedFunc);
    btAudio.setOnDisconnectCallback(disconnectedFunc);
    btAudio.start();
    while(1){
        ESP_LOGI(TAG, "%i\n", audioBytes);
        ESP_LOGI("volume", "%i\n", btAudio.getVolume());
        ESP_LOGI("connected", "%i\n", btAudio.getDeviceConnected());

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
