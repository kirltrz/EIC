#pragma once

#include <stdint.h>

#define TBD 0 // 待定参数
/******************************************************************************
 * 系统参数配置
 ******************************************************************************/
/* 调试配置 */
#define DEBUG_ENABLE 1 // 启用调试输出

#if DEBUG_ENABLE
#define DEBUG_LOG(...) Serial.print(__VA_ARGS__)
#else
#define DEBUG_LOG(...)
#endif
/******************************************************************************
 * 比赛规则配置
 ******************************************************************************/
#define LF_CORNER_START 1   // 是否可以从启停区左上角出发
#define GRAB_FREELY 1       // 是否可以自由抓取
#define TURNTABLE_HEIGHT 80 // 转盘高度mm

/******************************************************************************
 * 底盘参数配置
 ******************************************************************************/
/* 底盘运动参数 */
#define CHASSIS_MAX_SPEED TBD // 底盘最大速度mm/s
#define CHASSIS_MAX_OMEGA TBD // 底盘最大角速度rad/s
// 电机控制参数
#define ACC_VALUE 2000    // 位置模式加速度(RPM/s)
#define DEC_VALUE 2000    // 位置模式减速度(RPM/s)
#define MAX_VELOCITY 2400 // 位置模式最大速度(RPM)
/* 底盘PID参数 */

/* 底盘硬件参数 */
#define MOTOR_ACC TBD                // 电机加速度RPM/s（可更改，但需要与底盘PID参数匹配）
#define MOTOR_MAX_SPEED 2400         // 电机最大速度RPM（不可更改）
#define MOTOR_MIN_SPEED 0.1          // 电机最小速度RPM（不可更改）
#define WHEEL_RADIUS 41.0f           // 轮子半径，单位mm
#define ROBOT_RADIUS 106.5f          // 机器人中心到轮子的距离，单位mm
#define WHEEL_BASE_X 100.0f          // 小车X方向尺寸的一半(mm)
#define WHEEL_BASE_Y 100.0f          // 小车Y方向尺寸的一半(mm)
#define GEAR_RATIO 1.0f              // 电机与轮子的传动比
#define POSITION_TOLERANCE 10.0f     // 粗定位容差(mm)
#define FINE_POSITION_TOLERANCE 2.0f // 精定位容差(mm)
#define FINE_VELOCITY 500.0f         // 精定位模式下的最大速度(RPM)
#define FINE_VELOCITY_RAMP 1000      // 精定位速度斜率(RPM/s)
#define MIN_VELOCITY 0.1             // 精定位最小速度(RPM)
/******************************************************************************
 * 机械臂参数配置
 ******************************************************************************/
#define ARM_FIRST_JOINT_HEIGHT TBD // 机械臂第一关节距地面高度mm
#define ARM_FIRST_LENGTH TBD       // 机械臂第一关节至第二关节长度mm
#define ARM_SECOND_LENGTH TBD      // 机械臂第二关节至第三关节长度mm
#define ARM_THIRD_LENGTH TBD       // 机械臂第三关节至夹持点长度mm
#define ARM_MATERIAL_HEIGHT TBD    // 物料夹持点距地面高度mm

#define ARM_GRIPPER_OPEN_ANGLE TBD  // 机械臂夹爪张开角度
#define ARM_GRIPPER_CLOSE_ANGLE TBD // 机械臂夹爪闭合角度

/******************************************************************************
 * 硬件配置
 ******************************************************************************/
/* 电机 */
#define MOTOR_SERIAL Serial2
#define PIN_MOTOR_TX 9
#define PIN_MOTOR_RX 10
// 定义电机地址
#define MOTOR_BROADCAST 0 // 广播地址
#define MOTOR_FR 3        // 前右轮电机地址
#define MOTOR_FL 2        // 前左轮电机地址
#define MOTOR_BL 1        // 后左轮电机地址
#define MOTOR_BR 4        // 后右轮电机地址
// 反馈和控制周期(具体时间待调整)
#define CONTROL_INTERVAL 10  // 控制周期(ms)
#define FEEDBACK_INTERVAL 10 // 反馈周期(ms)
// 定位参数

/* 舵机 */
#define SERVO_SERIAL Serial1
#define PIN_SERVO_TX 14
#define PIN_SERVO_RX 13

/* PAW3395 */
#define PAW3395_OFFSET_X TBD // 安装偏移量X
#define PAW3395_OFFSET_Y TBD // 安装偏移量Y
#define PAW3395_SPI_HOST SPI3_HOST
#define PAW3395_DPI 26000
#define PIN_PAW3395_NRESET 2
#define PIN_PAW3395_NCS 42
#define PIN_PAW3395_SCLK 39
#define PIN_PAW3395_MISO 41
#define PIN_PAW3395_MOSI 40

/* HWT101 */
#define PIN_HWT101_SDA 5
#define PIN_HWT101_SCL 4

/* 视觉模块 （使用默认引脚，串口芯片上电即USB连接时视觉模块无法与主控通讯）*/
#define VISION_SERIAL Serial
#define PIN_VISION_TX SOC_TX0
#define PIN_VISION_RX SOC_RX0

/* 屏幕（触摸与屏幕共用SPI）*/
#define SCREEN_SPI_HOST SPI2_HOST
#define PIN_SCREEN_MISO 6
#define PIN_SCREEN_MOSI 7
#define PIN_SCREEN_CLK 15
#define PIN_SCREEN_DC 18
#define PIN_SCREEN_RST 3
#define PIN_SCREEN_CS 8
#define PIN_TOUCH_CS 16
#define PIN_TOUCH_INT 17

/******************************************************************************
 * 通信参数配置
 ******************************************************************************/
/* 串口通信 */
#define SERIAL_BAUDRATE 115200 // 通用串口波特率
#define VISION_TIMEOUT 1000    // 视觉通信超时时间(ms)

/******************************************************************************
 * 视觉模块命令
 ******************************************************************************/
/* 视觉命令类型 */
#define CMD_IDLE 0     // 待机模式
#define CMD_QRCODE 1   // 二维码识别
#define CMD_CIRCLE 2   // 色环识别
#define CMD_MATERIAL 3 // 物料识别

/* 颜色定义 */
#define COLOR_RED 1   // 红色
#define COLOR_GREEN 2 // 绿色
#define COLOR_BLUE 3  // 蓝色

/******************************************************************************
 * 错误代码定义
 ******************************************************************************/
#define ERROR_QRCODE_RECOGNITION_FAILED 1   // 二维码识别失败
#define ERROR_MATERIAL_RECOGNITION_FAILED 2 // 物料识别失败
#define ERROR_CIRCLE_RECOGNITION_FAILED 3   // 色环识别失败
