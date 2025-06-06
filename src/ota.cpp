#include "ota.h"
#include "displayInterface.h"
#include "taskManager.h"
#include "ui.h"
#include <Arduino.h>
#include "WiFi.h"
#include "ESPmDNS.h"
#include "WiFiUdp.h"
#include "ArduinoOTA.h"
#include "taskManager.h"
// OTA状态变量
static EIC_OTAState_t otaState = EIC_OTA_IDLE;
static bool otaInitialized = false;
static uint8_t otaProgress = 0;
static char otaStatusText[32] = "就绪";
static char wifiStatusText[32] = "未连接";
static char ipAddressText[32] = "0.0.0.0";
static bool wifiConnecting = false;
static const char *wifiSSID = NULL;
static const char *wifiPassword = NULL;
static const char *otaHostname = NULL;
static uint32_t restartTime = 0;         // 添加重启时间变量
static uint32_t lastWifiCheckTime = 0;   // 上次WiFi检查时间
static uint32_t wifiCheckInterval = 100; // WiFi检查间隔（100ms）
static bool useStaticIP = false;         // 是否使用静态IP
static IPAddress staticIP;               // 静态IP地址
static IPAddress staticGateway;          // 网关
static IPAddress staticSubnet;           // 子网掩码

// OTA事件处理回调
void setupOTACallbacks()
{
    ArduinoOTA.onStart([]()
                       {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else {
            type = "filesystem";
        }
        //Serial.println("开始OTA更新: " + type);
        otaState = EIC_OTA_RUNNING;
        otaProgress = 0;
        strcpy(otaStatusText, "正在更新...");
        // 切换到OTA屏幕
        lv_scr_load(ui_otaScreen); });

    ArduinoOTA.onEnd([]()
                     {
                         // Serial.println("\nOTA更新完成");
                         otaState = EIC_OTA_SUCCESS;
                         otaProgress = 100;
                         strcpy(otaStatusText, "更新完成,正在重启...");
                         restartTime = millis(); // 记录当前时间
                     });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                          {
        int percentage = (progress / (total / 100));
        //Serial.printf("OTA进度: %u%%\r", percentage);
        otaProgress = percentage; });

    ArduinoOTA.onError([](ota_error_t error)
                       {
        otaState = EIC_OTA_FAILED;
        //Serial.printf("OTA错误[%u]: ", error);
        
        if (error == OTA_AUTH_ERROR) {
            strcpy(otaStatusText, "认证失败");
            //Serial.println("认证失败");
        } else if (error == OTA_BEGIN_ERROR) {
            strcpy(otaStatusText, "开始失败");
            //Serial.println("开始失败");
        } else if (error == OTA_CONNECT_ERROR) {
            strcpy(otaStatusText, "连接失败");
            //Serial.println("连接失败");
        } else if (error == OTA_RECEIVE_ERROR) {
            strcpy(otaStatusText, "接收失败");
            //Serial.println("接收失败");
        } else if (error == OTA_END_ERROR) {
            strcpy(otaStatusText, "结束失败");
            //Serial.println("结束失败");
        } });
}
// 初始化OTA服务
void initOTAService()
{
    // 设置主机名
    ArduinoOTA.setHostname(otaHostname);

    // 设置事件回调
    setupOTACallbacks();

    // 开始OTA服务
    ArduinoOTA.begin();

    // 禁用自动重启
    ArduinoOTA.setRebootOnSuccess(false);

    // 标记OTA已初始化
    otaInitialized = true;
    strcpy(otaStatusText, "就绪");
}
// 初始化OTA（使用DHCP）
void initOTADHCP(const char *ssid, const char *password, const char *hostname)
{
    // 保存WiFi凭据
    wifiSSID = ssid;
    wifiPassword = password;
    otaHostname = hostname;
    useStaticIP = false;  // 使用DHCP

    // 配置WiFi模式
    WiFi.mode(WIFI_STA);

    // 开始连接WiFi
    WiFi.begin(wifiSSID, wifiPassword);
    wifiConnecting = true;
    strcpy(wifiStatusText, "正在连接...");

    // 释放信号量允许OTA任务开始工作
    xSemaphoreGive(xSemaphoreOTA);

    // Serial.println("OTA初始化完成");
}

// 初始化OTA（使用固定IP）
void initOTA(const char *ssid, const char *password, const char *hostname, IPAddress ip, IPAddress gateway, IPAddress subnet)
{
    // 保存WiFi凭据
    wifiSSID = ssid;
    wifiPassword = password;
    otaHostname = hostname;
    useStaticIP = true;  // 使用静态IP
    
    // 保存静态IP配置
    staticIP = ip;
    staticGateway = gateway;
    staticSubnet = subnet;

    // 配置WiFi模式
    WiFi.mode(WIFI_STA);
    
    // 连接WiFi前设置静态IP
    WiFi.config(staticIP, staticGateway, staticSubnet);

    // 开始连接WiFi
    WiFi.begin(wifiSSID, wifiPassword);
    wifiConnecting = true;
    strcpy(wifiStatusText, "正在连接...");

    // 释放信号量允许OTA任务开始工作
    xSemaphoreGive(xSemaphoreOTA);

    // Serial.println("OTA初始化完成(静态IP)");
}

// OTA任务
void otaTask(void *pvParameters)
{
    // 等待OTA初始化信号
    if (xSemaphoreOTA != NULL)
    {
        xSemaphoreTake(xSemaphoreOTA, portMAX_DELAY);
        // 不需要释放信号量，因为这只是一个启动信号
    }

    // 任务主循环
    for (;;)
    {
        uint32_t currentTime = millis();

        // 检查是否需要重启
        if (otaState == EIC_OTA_SUCCESS && restartTime != 0)
        {
            if (currentTime - restartTime >= 500)
            {                  // 等待0.5秒
                ESP.restart(); // 重启设备
            }
        }

        // 定期检查WiFi状态
        if (currentTime - lastWifiCheckTime >= wifiCheckInterval)
        {
            lastWifiCheckTime = currentTime;

            if (WiFi.status() == WL_CONNECTED)
            {
                if (wifiConnecting)
                {
                    // 首次连接成功
                    wifiConnecting = false;
                    strcpy(wifiStatusText, "已连接");
                    strcpy(ipAddressText, WiFi.localIP().toString().c_str());

                    // 初始化或重新初始化OTA服务
                    if (!otaInitialized)
                    {
                        initOTAService();
                    }
                }

                // 处理OTA请求
                ArduinoOTA.handle();
            }
            else
            {
                // WiFi未连接
                if (!wifiConnecting)
                {
                    // 开始重连
                    WiFi.disconnect();
                    delay(100);
                    
                    // 如果使用静态IP，需要重新配置
                    if (useStaticIP) {
                        WiFi.config(staticIP, staticGateway, staticSubnet);
                    }
                    
                    WiFi.begin(wifiSSID, wifiPassword);
                    wifiConnecting = true;
                    strcpy(wifiStatusText, "正在重连...");
                    strcpy(ipAddressText, "0.0.0.0");
                    strcpy(otaStatusText, "WiFi断开");
                    otaInitialized = false; // 重置OTA初始化状态
                }
            }
        }

        // 延时以降低CPU使用率
        delay(10);
    }
}

// 获取当前OTA状态
EIC_OTAState_t getOTAState()
{
    return otaState;
}

// 更新OTA UI的函数，由LVGL定时器调用
void updateOTAUI(void)
{
    if (otaInitialized)
    {
        lv_bar_set_value(ui_otaPercent, otaProgress, LV_ANIM_OFF);
        lv_label_set_text(ui_otaStatusLabel, otaStatusText);
        lv_label_set_text(ui_wifiStatusLabel, wifiStatusText);
        lv_label_set_text(ui_ipAddressLabel, ipAddressText);

        // 根据OTA状态设置进度条样式
        switch (otaState)
        {
        case EIC_OTA_RUNNING:
            // 下载中状态
            lv_obj_set_style_bg_color(ui_otaPercent, lv_color_hex(0xDAFAEB), LV_PART_MAIN);
            lv_obj_set_style_bg_color(ui_otaPercent, lv_color_hex(0x6BEDB6), LV_PART_INDICATOR);
            break;

        case EIC_OTA_SUCCESS:
            // 完成状态：极光绿 #00C853
            lv_obj_set_style_bg_color(ui_otaPercent, lv_color_hex(0xE8F5E9), LV_PART_MAIN);
            lv_obj_set_style_bg_color(ui_otaPercent, lv_color_hex(0x00C853), LV_PART_INDICATOR);
            break;

        case EIC_OTA_FAILED:
            // 错误/中断状态：警示红 #D50000
            lv_obj_set_style_bg_color(ui_otaPercent, lv_color_hex(0xFFEBEE), LV_PART_MAIN);
            lv_obj_set_style_bg_color(ui_otaPercent, lv_color_hex(0xD50000), LV_PART_INDICATOR);
            break;

        default:
            // 默认状态，与下载中状态相同
            lv_obj_set_style_bg_color(ui_otaPercent, lv_color_hex(0xDAFAEB), LV_PART_MAIN);
            lv_obj_set_style_bg_color(ui_otaPercent, lv_color_hex(0x6BEDB6), LV_PART_INDICATOR);
            break;
        }
    }
}