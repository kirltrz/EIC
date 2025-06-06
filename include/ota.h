#pragma once
#include <IPAddress.h>
// OTA状态
typedef enum {
    EIC_OTA_IDLE,
    EIC_OTA_RUNNING,
    EIC_OTA_SUCCESS,
    EIC_OTA_FAILED
} EIC_OTAState_t;

// OTA配置
void initOTADHCP(const char* ssid="gcs", const char* password="abababab", const char* hostname = "ESP32-OTA");

// 使用静态IP的OTA初始化函数（其他参数自动获取）
void initOTA(const char *ssid="gcs", const char *password="abababab", const char *hostname = "ESP32-OTA", 
             IPAddress ip=IPAddress(192,168,137,228), 
             IPAddress gateway=IPAddress(192,168,137,1), 
             IPAddress subnet=IPAddress(255,255,255,0));

// OTA任务
void otaTask(void* pvParameters);

// 更新OTA UI
void updateOTAUI(void);
// 获取OTA状态
EIC_OTAState_t getOTAState();
