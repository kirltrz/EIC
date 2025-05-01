#include <Arduino.h>
#include "config.h"
#include "displayInterface.h"
#include "taskManager.h"
#include "sensor.h"
#include "LED.h"
#include "motion.h"
#include "vision.h"
#include "arm.h"

void setup()
{
    #if DEBUG_ENABLE
    Serial.begin(115200);
    #endif

    initMotor();
    initArm();
    initLED();
    initSensor();
    initGUI();
    initTaskManager();
}

void loop()
{
    delay(1);//让出CPU时间
}

