#pragma once

#include <stdint.h>

#define TBD 0 // 待定参数
/******************************************************************************
 * 系统参数配置
 ******************************************************************************/
/* 调试配置 */
// #define DEBUG_ENABLE 0 // 启用调试输出，在platformio.ini中配置
#define DEBUG_SERIAL Serial

#if DEBUG_ENABLE
#define DEBUG_LOG(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_LOG(...) //Serial.printf(__VA_ARGS__)
#endif
/******************************************************************************
 * 比赛规则配置
 ******************************************************************************/
#define LF_CORNER_START 1   // 是否可以从启停区左上角出发
#define GRAB_FREELY 1       // 是否可以自由抓取
#define TURNTABLE_HEIGHT 80 // 转盘高度mm
#define MATERIAL_HEIGHT 70 // 物料高度mm
/******************************************************************************
 * 底盘参数配置
 ******************************************************************************/
/* 底盘运动参数 */
#define CHASSIS_MAX_SPEED TBD // 底盘最大速度mm/s
#define CHASSIS_MAX_OMEGA TBD // 底盘最大角速度rad/s
// 电机控制参数
#define DEFAULT_SPEED 100.0f
#define DEFAULT_ACC 180 //缓启动加速度，范围0-255，0为直接启动
/* 底盘PID参数 */
// PID控制参数 - 运动中使用
extern float POS_KP;     // 位置环比例系数
extern float POS_KI;      // 位置环积分系数
extern float POS_KD;     // 位置环微分系数
extern float YAW_KP;     // 偏航角比例系数
extern float YAW_KI;      // 偏航角积分系数
extern float YAW_KD;     // 偏航角微分系数

// PID控制参数 - 位置锁定时使用（比例系数更高，以增强保持力）
extern float HOLD_POS_KP;    // 位置保持比例系数
extern float HOLD_POS_KI;     // 完全禁用积分作用
extern float HOLD_POS_KD;     // 位置保持微分系数
extern float HOLD_YAW_KP;    // 偏航角保持比例系数
extern float HOLD_YAW_KI;     // 完全禁用积分作用
extern float HOLD_YAW_KD;     // 偏航角保持微分系数

// 运动控制参数
extern float MAX_LINEAR_SPEED;  // 最大线速度，单位mm/s - 大幅增加
extern float MAX_ANGULAR_SPEED;  // 最大角速度，单位度/s - 大幅增加

#define MIN_SPEED_RPM 0.1f       // 电机最小速度，单位RPM - 常量，取决于硬件不可更改
#define MAX_SPEED_RPM 2400.0f    // 最大速度，单位RPM - 常量，取决于硬件不可更改

// 位置保持模式参数
extern float HOLD_MAX_LINEAR_SPEED;  // 位置保持时最大线速度，单位mm/s
extern float HOLD_MAX_ANGULAR_SPEED;  // 位置保持时最大角速度，单位度/s
/* 底盘硬件参数 */
// 全向轮底盘参数
#define WHEEL_RADIUS 41.0f    // 轮子半径，单位mm
#define ROBOT_RADIUS 106.5f   // 机器人中心到轮子的距离，单位mm
/******************************************************************************
 * 机械臂参数配置
 ******************************************************************************/
#define ARM_FIRST_JOINT_OFFSET_W 20 // 机械臂第一关节在W轴方向偏移量
#define ARM_FIRST_JOINT_HEIGHT 139  // 机械臂第一关节距地面高度mm
#define ARM_FIRST_LENGTH 185        // 机械臂第一关节至第二关节长度mm
#define ARM_SECOND_LENGTH 185       // 机械臂第二关节至第三关节长度mm
#define ARM_THIRD_LENGTH 112        // 机械臂第三关节至夹持点长度mm
#define ARM_MATERIAL_HEIGHT 56     // 物料夹持点距地面高度mm
#define ARM_MATERIAL_OFFSET_X 2  // 物料夹持点距机械臂中心x轴偏移量mm
#define ARM_MATERIAL_OFFSET_W -27.5  // 物料夹持点距机械臂中心w轴偏移量mm

#define BASIS_ARM_ANGLE_MIN -180.0f    // 云台舵机最小角度
#define BASIS_ARM_ANGLE_MAX 180.0f  // 云台舵机最大角度
#define FIRST_ARM_ANGLE_MIN -57.0f  // 大臂最小角度
#define FIRST_ARM_ANGLE_MAX 100.0f   // 大臂最大角度
#define SECOND_ARM_ANGLE_MIN -110.0f // 小臂最小角度
#define SECOND_ARM_ANGLE_MAX 60.0f  // 小臂最大角度
#define THIRD_ARM_ANGLE_MIN -90.0f
#define THIRD_ARM_ANGLE_MAX 45.0f

#define ARM_GRIPPER_OPEN_ANGLE 40.0f  // 机械臂夹爪张开角度
#define ARM_GRIPPER_CLOSE_ANGLE -7.0f // 机械臂夹爪闭合角度

// 夹爪位置监测配置
#define GRIPPER_HOLD_ANGLE_THRESHOLD -6.5f    // 夹持检测角度阈值，单位度，大于此值为夹持状态
#define GRIPPER_OPEN_ANGLE_THRESHOLD 35.0f    // 张开状态角度阈值，单位度，接近此值为张开状态
#define GRIPPER_MONITOR_INTERVAL 100          // 夹爪监测任务更新间隔，单位ms
#define GRIPPER_STABLE_COUNT 3                // 连续稳定检测次数，用于避免误判
#define GRIPPER_ANGLE_TOLERANCE 0.5f          // 角度检测容差，单位度

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
// 定位参数

/* 舵机 */
#define SERVO_SERIAL Serial1
#define PIN_SERVO_TX 13
#define PIN_SERVO_RX 14

/* PAW3395 光学传感器配置 */
// 传感器位置偏移配置（相对于小车中心的偏移，单位：mm）
/*补偿个鸡毛不补偿了，又是微分又是积分还有三角函数，反正两圈下来角度都一样*/
#define PAW3395_OFFSET_X 0.0f
#define PAW3395_OFFSET_Y 0.0f

// 传感器方向配置（相对于小车坐标系的方向变换）
// 根据实际测试：小车y+→传感器x-，小车x+→传感器y+
#define PAW3395_DIRECTION_X_SCALE -1.0f   // X方向缩放因子，1表示正向
#define PAW3395_DIRECTION_Y_SCALE 1.0f   // Y方向缩放因子，1表示正向
#define PAW3395_SWAP_XY true             // 交换X和Y轴，true表示传感器旋转90度安装

// 硬件引脚配置
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
#define MOTOR_BAUDRATE 921600 // 电机串口波特率
#define VISION_TIMEOUT 3000    // 视觉超时时间(ms)

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

#define VISION_GET_MATERIAL_TIMEOUT 5000 //物料识别超时时间
