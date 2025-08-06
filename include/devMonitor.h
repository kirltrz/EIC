#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// 进度更新控制开关
// 设置为 1 启用进度更新，设置为 0 禁用进度更新
#define PROGRESS_UPDATE_ENABLE 1

#if PROGRESS_UPDATE_ENABLE
    #define P(progress) updateTaskProgress(progress)
#else
    #define P(progress) ((void)0)
#endif

struct deviceStatus_t
{
    bool hwt101_status;
    bool paw3395_status;
    bool vision_status;
    bool motor_status;
    bool servo_status;
};

/// @brief 任务进度枚举
enum taskProgress_t {
    TASK_IDLE = 0,           // 待机状态
    TASK_RESET_SENSOR,       // 重置定位系统
    TASK_SCAN_QRCODE,        // 扫描二维码
    TASK_FIRST_TURNTABLE,    // 第一轮：转盘抓取
    TASK_FIRST_ROUGH,        // 第一轮：粗加工区
    TASK_FIRST_STORAGE,      // 第一轮：暂存区
    TASK_SECOND_TURNTABLE,   // 第二轮：转盘抓取
    TASK_SECOND_ROUGH,       // 第二轮：粗加工区
    TASK_SECOND_STORAGE,     // 第二轮：暂存区
    TASK_RETURN_HOME,        // 回到启停区
    TASK_COMPLETED           // 任务完成
};

/// @brief 设备状态变量
extern deviceStatus_t devStatus;

/// @brief 设备状态互斥锁
extern SemaphoreHandle_t devStatusMutex;

/// @brief 任务进度变量
extern taskProgress_t taskProgress;

/// @brief 任务进度互斥锁
extern SemaphoreHandle_t taskProgressMutex;

/// @brief 获取设备状态的副本（线程安全）
/// @param status 用于存储设备状态的指针
void getDeviceStatus(deviceStatus_t *status);

/// @brief 更新任务进度
/// @param progress 新的任务进度
void updateTaskProgress(taskProgress_t progress);

/// @brief 获取任务进度百分比
/// @return 进度百分比 (0-100)
uint8_t getTaskProgressPercent(void);

/// @brief 检查所有设备并更新状态
/// @return 所有设备是否都正常工作
bool checkDevice(void);

/// @brief 检查位置数据是否锁死
/// @return 位置数据是否正常
bool checkPositionData(void);

/// @brief 初始化设备监控
void initDevMonitor(void);
