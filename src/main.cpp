#include <Arduino.h>
#include "config.h"
#include "displayInterface.h"
#include "taskManager.h"
#include "sensor.h"
#include "LED.h"
#include "motion.h"
#include "vision.h"
#include "arm.h"
#include "ota.h"
#include "devMonitor.h"

void setup()
{
    #if DEBUG_ENABLE
    DEBUG_SERIAL.begin(115200);
    #endif

    initArm();
    initLED();
    initSensor();
    initMotor();//电机驱动板初始化较慢，为确保上电后失能电机，让其他部分先初始化以等待
    initGUI();

    delay(500); // 等待初始化完成

    initTaskManager();
    
    #if DEBUG_ENABLE
    initOTA();
    #else
    initDevMonitor();
    #endif
}

void loop()
{
    delay(1);//让出CPU时间
}

