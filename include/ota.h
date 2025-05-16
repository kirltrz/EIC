#pragma once

// OTA状态
typedef enum {
    EIC_OTA_IDLE,
    EIC_OTA_RUNNING,
    EIC_OTA_SUCCESS,
    EIC_OTA_FAILED
} EIC_OTAState_t;

// OTA配置
void initOTA(const char* ssid="gcs", const char* password="abababab", const char* hostname = "ESP32-OTA");

// OTA任务
void otaTask(void* pvParameters);

// 更新OTA UI
void updateOTAUI(void);
// 获取OTA状态
EIC_OTAState_t getOTAState();
