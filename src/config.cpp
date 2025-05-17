#include "config.h"

// PID控制参数 - 运动中使用
float POS_KP = 30.0f;      // 位置环比例系数
float POS_KI = 0.1f;       // 位置环积分系数
float POS_KD = 15.0f;      // 位置环微分系数
float YAW_KP = 50.0f;      // 偏航角比例系数
float YAW_KI = 0.1f;       // 偏航角积分系数
float YAW_KD = 15.0f;      // 偏航角微分系数

// PID控制参数 - 位置锁定时使用
float HOLD_POS_KP = 2.0f;    // 位置保持比例系数
float HOLD_POS_KI = 0.4f;     // 位置保持积分系数
float HOLD_POS_KD = 0.8f;    // 位置保持微分系数
float HOLD_YAW_KP = 3.0f;   // 偏航角保持比例系数
float HOLD_YAW_KI = 0.2f;     // 偏航角保持积分系数
float HOLD_YAW_KD = 1.0f;    // 偏航角保持微分系数

// 运动控制参数
float MAX_LINEAR_SPEED = 15000.0f;  // 最大线速度，单位mm/s
float MAX_ANGULAR_SPEED = 720.0f;   // 最大角速度，单位度/s

// 位置保持模式参数
float HOLD_MAX_LINEAR_SPEED = 5000.0f;  // 位置保持时最大线速度，单位mm/s
float HOLD_MAX_ANGULAR_SPEED = 720.0f;  // 位置保持时最大角速度，单位度/s 