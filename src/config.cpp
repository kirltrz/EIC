#include "config.h"

// PID控制参数 - 运动中使用
float POS_KP = 1.0f;      // 位置环比例系数
float POS_KI = 0.0f;       // 位置环积分系数
float POS_KD = 0.0f;      // 位置环微分系数
float YAW_KP = 1.0f;      // 偏航角比例系数
float YAW_KI = 0.0f;       // 偏航角积分系数
float YAW_KD = 0.0f;      // 偏航角微分系数

// PID控制参数 - 位置锁定时使用
float HOLD_POS_KP = 1.0f;    // 位置保持比例系数
float HOLD_POS_KI = 0.0f;     // 位置保持积分系数
float HOLD_POS_KD = 0.0f;    // 位置保持微分系数
float HOLD_YAW_KP = 1.0f;   // 偏航角保持比例系数
float HOLD_YAW_KI = 0.0f;     // 偏航角保持积分系数
float HOLD_YAW_KD = 0.0f;    // 偏航角保持微分系数

// 运动控制参数
float MAX_LINEAR_SPEED = 1000.0f;  // 最大线速度，单位mm/s
float MAX_ANGULAR_SPEED = 180.0f;   // 最大角速度，单位度/s

// 位置保持模式参数
float HOLD_MAX_LINEAR_SPEED = 500.0f;  // 位置保持时最大线速度，单位mm/s
float HOLD_MAX_ANGULAR_SPEED = 90.0f;  // 位置保持时最大角速度，单位度/s 