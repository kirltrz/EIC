#pragma once

#include <stdint.h>

/******************************************************************************
 * 系统参数配置
 ******************************************************************************/
/* 系统版本 */
#define SYSTEM_VERSION        "0.1.0"

/* 调试配置 */
#define DEBUG_ENABLE          1            // 启用调试输出
#define DEBUG_SERIAL_RATE     115200       // 调试串口波特率

/******************************************************************************
 * 硬件配置
 ******************************************************************************/
/* 电机引脚 */
#define MOTOR_SERIAL          Serial1
#define PIN_MOTOR_TX          1
#define PIN_MOTOR_RX          0

/* 舵机引脚 */
#define PIN_SERVO_TX          2
#define PIN_SERVO_RX          3

/* PAW3395引脚 */
#define PIN_PAW3395_NRESET    4
#define PIN_PAW3395_NCS       5
#define PIN_PAW3395_SCLK      6
#define PIN_PAW3395_MISO      7
#define PIN_PAW3395_MOSI      8

/* HWT101引脚 */
#define PIN_HWT101_SDA        9
#define PIN_HWT101_SCL        10

/* 视觉模块引脚 （使用默认引脚，串口芯片上电即USB连接时视觉模块无法与主控通讯）*/
#define VISION_SERIAL         Serial
#define PIN_VISION_TX         SOC_TX0
#define PIN_VISION_RX         SOC_RX0

/* 屏幕引脚（触摸与屏幕共用SPI）*/
#define SCREEN_SPI_HOST       SPI2_HOST
#define PIN_SCREEN_MISO       6
#define PIN_SCREEN_MOSI       7
#define PIN_SCREEN_CLK        15
#define PIN_SCREEN_DC         18
#define PIN_SCREEN_RST        3
#define PIN_SCREEN_CS         8
#define PIN_TOUCH_CS          16
#define PIN_TOUCH_INT         17

/******************************************************************************
 * 通信参数配置
 ******************************************************************************/
/* 串口通信 */
#define SERIAL_BAUDRATE       115200       // 通用串口波特率
#define VISION_TIMEOUT        1000         // 视觉通信超时时间(ms)

/******************************************************************************
 * 视觉模块命令
 ******************************************************************************/
/* 视觉命令类型 */
#define CMD_IDLE              0            // 待机模式
#define CMD_QRCODE            1            // 二维码识别
#define CMD_CIRCLE            2            // 色环识别
#define CMD_MATERIAL          3            // 物料识别

/* 颜色定义 */
#define COLOR_RED             1            // 红色
#define COLOR_GREEN           2            // 绿色
#define COLOR_BLUE            3            // 蓝色

/******************************************************************************
 * 错误代码定义
 ******************************************************************************/
#define ERROR_NONE            0            // 无错误
#define ERROR_TIMEOUT         1            // 超时错误
#define ERROR_COMMUNICATION   2            // 通信错误
#define ERROR_SENSOR          3            // 传感器错误
#define ERROR_MOTION          4            // 运动错误
#define ERROR_VISION          5            // 视觉错误
#define ERROR_TASK            6            // 任务错误

