#include "mainSequence.h"
#include "motion.h"
#include "arm.h"
#include "vision.h"
#include "errorHandler.h"
#include "config.h"
#include "sensor.h"
#include "VOFAdebug.h"

#if !DEBUG_ENABLE
#include "ui.h"
#endif

#define TASK_TIMEOUT 5000
POS pos[] = {
    {-153.0  , 576.0  , 0.0f  }, // 0扫二维码
    {  0.0   , 1420.0 , 0.0f  }, // 1转盘抓取
    {-194.0  , 1000.0 , 90.0f }, // 2离开转盘
    {-1940.0 , 990.0  , 90.0f }, // 3粗加工区
    {-1853.0 , 1824.0 , 90.0f }, // 4离开粗加工区
    {-1038.0 , 1957.0 , 0.0f  }, // 5暂存区
    {-143.0  , 1857.0 , 0.0f  }, // 6离开暂存区（准备第二轮到转盘）
    {-1021.0 , 187.0  , 0.0f  }, // 7离开暂存区（准备回到启停区）
    {-132.0  , 190.0  , 0.0f  }, // 8准备进入启停区
    {50.0    , -20.0  , 0.0f  }, // 9回到启停区
};

POS que[] = {
    {-153.0  , 576.0  , 0.0f  }, // 0扫二维码
    {0.0     , 1420.0 , 0.0f  }, // 1转盘抓取
    {-194.0  , 1000.0 , 90.0f }, // 2离开转盘
    {-1940.0 , 1017.0 , 90.0f }, // 3粗加工区
    {-1853.0 , 1824.0 , 15.0f }, // 4离开粗加工区
    {-1008.0 , 1957.0 , 0.0f  }, // 5暂存区
    {-143.0  , 1857.0 , 0.0f  }, // 6离开暂存区（准备第二轮到转盘）
    {0.0     , 1420.0 , 0.0f  }, // 1转盘抓取
    {-194.0  , 1000.0 , 90.0f }, // 2离开转盘
    {-1940.0 , 1017.0 , 90.0f }, // 3粗加工区
    {-1853.0 , 1824.0 , 15.0f }, // 4离开粗加工区
    {-1008.0 , 1957.0 , 0.0f  }, // 5暂存区
    {-1021.0 , 187.0  , 0.0f  }, // 7离开暂存区（准备回到启停区）
    {-132.0  , 190.0  , 0.0f  }, // 8准备进入启停区
    {0.0     , 0.0    , 0.0f  }, // 9回到启停区
};
int taskcode[2][3] = {0};
int circleOffset[3][2] = {0};
int startTime = 0;
void startMainSequence(void)
{
    /*启动主流程*/
    // 直接释放信号量，通知mainSequenceTask可以开始运行
    xSemaphoreGive(xSemaphoreMainsequence);
}

void updateTaskcodeDisplay(void)
{
    // 格式化任务码为"123+456"的形式
    char taskcodeStr[10];
    snprintf(taskcodeStr, sizeof(taskcodeStr), "%d%d%d+%d%d%d", 
             taskcode[0][0], taskcode[0][1], taskcode[0][2],
             taskcode[1][0], taskcode[1][1], taskcode[1][2]);
    
#if DEBUG_ENABLE
    // Debug模式：输出到串口
    DEBUG_LOG("任务码: %s\n", taskcodeStr);
#else
    // Release模式：更新UI显示
    if (ui_taskcodeText != NULL) {
        lv_label_set_text(ui_taskcodeText, taskcodeStr);
    }
#endif
}

void mainSequenceTask(void *pvParameters)
{
    // 等待信号量，表示主流程可以开始
    if (xSemaphoreTake(xSemaphoreMainsequence, portMAX_DELAY) == pdTRUE)
    {
        unsigned long startTime = millis();
        // 主流程的代码
        resetSensor();    // 重置定位系统到 0, 0, 0
        DEBUG_LOG("定位系统已重置到 0, 0, 0");
        P(TASK_RESET_SENSOR);

        motor->enControl(MOTOR_BROADCAST, true);//使能电机，准备运动
        delay(500);

        moveTo(pos[0]);   // 前往二维码前方
        arm_ScanQRcode(); // 机械臂运动至扫描二维码状态
        waitNear();

        // QR码扫描逻辑
        if (!visionScanQRcode(taskcode[0], taskcode[1])) {
            errorHandle(ERROR_QRCODE_RECOGNITION_FAILED);
        }
        
        // 获取到任务码后更新UI显示
        updateTaskcodeDisplay();
        P(TASK_SCAN_QRCODE);

        moveTo({pos[1].x-20.0, pos[1].y, pos[1].yaw}); // 1 先靠近转盘，防止由于误差而超出边界
        //waitNear();
        caliTurntable(); // 根据转盘校准车身
        arm_catchFromTurntable(taskcode[0]);
        P(TASK_FIRST_TURNTABLE);
        moveTo(pos[2]); // 1 前往离开转盘状态
        waitNear();
        
        moveTo(pos[3]); // 1 前往粗加工区
        waitArrived();
        caliCircle(pos[3], CALI_ROUGH);
        arm_putToGround(taskcode[0]);
        arm_catchFromGround(taskcode[0], 1);
        P(TASK_FIRST_ROUGH);
        moveTo(pos[4]); // 1 离开粗加工区
        waitNear();
        moveTo({pos[4].x, pos[4].y, 0.0f});
        waitArrived();
        
        moveTo({pos[5].x, pos[5].y-20, pos[5].yaw}); // 1 前往暂存区
        waitArrived();
        caliCircle(pos[5], CALI_ROUGH);
        arm_putToGround(taskcode[0]);
        P(TASK_FIRST_STORAGE);
        moveTo(pos[6]); // 1 离开暂存区
        waitNear();
        
        moveTo({pos[1].x-30, pos[1].y, pos[1].yaw}); // 2 先靠近转盘，防止由于误差而超出边界
        waitNear();
        caliTurntable(); // 根据转盘校准车身
        arm_catchFromTurntable(taskcode[1]);
        P(TASK_SECOND_TURNTABLE);
        moveTo(pos[2]); // 2 前往离开转盘状态
        waitNear();
        
        moveTo(pos[3]); // 2 前往粗加工区
        waitArrived();
        caliCircle(pos[3], CALI_ROUGH);
        arm_putToGround(taskcode[1]);
        arm_catchFromGround(taskcode[1], 2);
        P(TASK_SECOND_ROUGH);
        moveTo(pos[4]); // 2 离开粗加工区
        waitNear();
        moveTo({pos[4].x, pos[4].y, 0.0f});
        waitArrived();
        
        moveTo({pos[5].x, pos[5].y-20, pos[5].yaw}); // 2 前往暂存区
        waitArrived();
        caliCircle(pos[5], CALI_STACKING); // 码垛时，机械臂在更高高度校准
        arm_putToMaterial(taskcode[1]);
        P(TASK_SECOND_STORAGE);
        moveTo(pos[5]); // 先归中防止压线
        waitNear();
        moveTo(pos[7]); // 2 离开暂存区
        waitNear();
        
        moveTo(pos[8]); // 前往启停区
        P(TASK_RETURN_HOME);
        waitNear();
        moveTo(pos[9]); // 回到启停区
        waitArrived();
        
        P(TASK_COMPLETED);
        motor->enControl(MOTOR_BROADCAST, false);
        visionToIDLE();
        UDP_LOG("主流程用时%d分%d秒", (millis() - startTime)/60000, (millis() - startTime)%60000/1000);
        // 释放信号量，表示主流程已完成
        xSemaphoreGive(xSemaphoreMainsequence);
    }
    
    vTaskDelete(NULL); // 删除当前任务
}

void caliTurntable(void){
    arm_turntableDetect();
        
    // 根据摄像头返回的与转盘中心的差值逐步靠近转盘
    int x, y;
    const float scale = 0.5f;
    const int threshold = 15; // 阈值，当摄像头返回的数值小于此值时终止校准
    const int timeout_ms = 5000; // 超时时间
    int start_time = millis();
    
    global_position_t current_pos;
    POS current_pos_xy;
    
    while (millis() - start_time < timeout_ms) {
        if (visionGetTurntable(&x, &y)) {
            // 根据视觉反馈调整位置
            getGlobalPosition(&current_pos);
            current_pos_xy = {current_pos.x, current_pos.y, pos[1].yaw};
            current_pos_xy.x += y * scale * 0.9f;
            current_pos_xy.y += -x * scale * 0.9f;
            moveTo(current_pos_xy);
            // 检查是否达到阈值
            if (abs(x) < threshold && abs(y) < threshold) {
                //DEBUG_LOG("转盘校准完成，误差在阈值内: x=%d, y=%d", x, y);
                break;
            }
        } else {
            //DEBUG_LOG("获取转盘位置失败，继续尝试");
            delay(200);
        }
    }
    
    if (millis() - start_time >= timeout_ms) {
        //DEBUG_LOG("转盘校准超时");
    }
    
    // 设置全局定位xy数据为pos[1]中的数据
    setGlobalPosition(pos[1].x, pos[1].y-10.0f);
    moveTo(pos[1]);
}

/**
 * @brief 校准色环
 * @param caliBase 校准基准点，即中间色环所在的位置
 * @param mode 校准模式：CALI_ROUGH(粗定位), CALI_PRECISE(精确定位), CALI_STACKING(码垛定位)
 */
void caliCircle(POS caliBase, CaliMode mode)
{
    UDP_LOG("在%lums开始色环校准，模式：%d", millis(), mode);
    
    // 根据摄像头返回的与转盘中心的差值逐步靠近转盘
    int x, y, color;
    
    // 根据模式设置参数
    float scale, ki;
    int threshold;
    bool use_integral;
    
    switch (mode) {
        case CALI_ROUGH:      // 粗定位（机械臂标准位）
            scale = 0.2f;
            ki = 0.008f;
            threshold = 5;
            use_integral = true;
            break;
            
        case CALI_PRECISE:    // 精确定位（机械臂低位）
            scale = 0.3f;
            ki = 0.01f;       // 精确定位使用积分项
            threshold = 10;
            use_integral = true;
            arm_groundDetect(2, false);  // 降低机械臂
            break;
            
        case CALI_STACKING:   // 码垛定位（粗定位+机械臂高位）
            scale = 0.3f;
            ki = 0.0f;        // 码垛定位不使用积分项
            threshold = 20;
            use_integral = false;
            // 机械臂保持高位，不需要额外调用
            break;
    }
    
    const float integral_limit = 100.0f; // 积分项限制，防止积分饱和
    float integral_x = 0.0f;        // x方向积分累积
    float integral_y = 0.0f;        // y方向积分累积
    const int timeout_ms = 5000; // 超时时间
    const int camera_offset = 11.0f; // 摄像头安装偏移量
    int start_time = millis();
    
    // 退出条件时间控制：需要持续500ms满足条件才退出
    unsigned long exit_condition_start_time = 0;
    const int required_stable_duration_ms = 1000;
    bool is_stable = false;
    bool isRoughArea = false;

    if (caliBase.x == pos[3].x && caliBase.y == pos[3].y && caliBase.yaw == pos[3].yaw)
    {
        isRoughArea = true;
    }
    else if (caliBase.x == pos[5].x && caliBase.y == pos[5].y && caliBase.yaw == pos[5].yaw)
    {
        isRoughArea = false;
    }

    global_position_t current_pos;
    POS current_pos_xy;

    while (millis() - start_time < timeout_ms)
    {
        if (visionGetCircle(&x, &y))
        {
            //sendDebugValuesUDP(x, y, integral_x, integral_y);
            
            // 检查是否达到阈值（需要持续500ms满足条件才退出）
            if (abs(x) < threshold && abs(y) < threshold)
            {
                if (!is_stable)
                {
                    // 首次满足条件，记录开始时间
                    exit_condition_start_time = millis();
                    is_stable = true;
                    // DEBUG_LOG("开始满足校准条件: x=%d, y=%d", x, y);
                }
                else
                {
                    // 检查是否已持续满足条件足够时间
                    unsigned long stable_duration = millis() - exit_condition_start_time;
                    if (stable_duration >= required_stable_duration_ms)
                    {
                        UDP_LOG("退出时误差：x=%d, y=%d，持续时间：%lums", x, y, stable_duration);
                        // DEBUG_LOG("校准完成，持续满足条件%dms", stable_duration);
                        break;
                    }
                    // DEBUG_LOG("校准条件持续满足中: %dms/%dms", stable_duration, required_stable_duration_ms);
                }
                // 满足条件但还未达到要求时间时，暂停移动避免干扰
                delay(50);
                continue;
            }
            else
            {
                // 不满足条件时重置状态
                is_stable = false;
                exit_condition_start_time = 0;
            }
            
            // 累积积分项 (I) 并限制积分饱和（仅在使用积分项时）
            if (use_integral) {
                integral_x += x;
                integral_y += y;
                
                // 积分限制，防止积分饱和
                if (integral_x > integral_limit) integral_x = integral_limit;
                if (integral_x < -integral_limit) integral_x = -integral_limit;
                if (integral_y > integral_limit) integral_y = integral_limit;
                if (integral_y < -integral_limit) integral_y = -integral_limit;
            }
            
            // 根据视觉反馈调整位置 (P 或 P + I 控制)
            getGlobalPosition(&current_pos);
            current_pos_xy = {current_pos.x, current_pos.y, caliBase.yaw};
            
            float correction_x, correction_y;
            if(isRoughArea)
            {
                // 比例项 + 积分项（如果启用）
                correction_x = -y * scale + (use_integral ? (-integral_y * ki) : 0.0f);
                correction_y = x * scale + (use_integral ? (integral_x * ki) : 0.0f);
                current_pos_xy.x += correction_x;
                current_pos_xy.y += correction_y;
            }
            else
            {
                // 比例项 + 积分项（如果启用）
                correction_x = x * scale + (use_integral ? (integral_x * ki) : 0.0f);
                correction_y = y * scale + (use_integral ? (integral_y * ki) : 0.0f);
                current_pos_xy.x += correction_x;
                current_pos_xy.y += correction_y;
            }
            
            // 根据模式设置运动参数
            bool high_precision = (mode == CALI_PRECISE);
            moveTo(current_pos_xy, high_precision);
        }
        else
        {
            // 获取位置失败时重置状态
            is_stable = false;
            exit_condition_start_time = 0;
            // DEBUG_LOG("获取位置失败，继续尝试");
            delay(100);
        }
    }

    if (millis() - start_time >= timeout_ms)
    {
        if (is_stable)
        {
            unsigned long stable_duration = millis() - exit_condition_start_time;
            UDP_LOG("校准超时，已持续满足条件：%lums/%dms", stable_duration, required_stable_duration_ms);
        }
        else
        {
            UDP_LOG("校准超时，未满足退出条件");
        }
    }
    UDP_LOG("在%lums结束色环校准用时%lums，开始颜色识别", millis(), millis() - start_time);
    start_time = millis();
    visionGetCircleColor(&color);
    
    UDP_LOG("在%lums结束颜色识别用时%lums", millis(), millis() - start_time);
    //UDP_LOG("color: %d，isRoughArea: %d", color, isRoughArea);
    //motor->enControl(MOTOR_BROADCAST, false);//防止车身在重置位置的时候乱动
    if (color == 1)
    {
        if (isRoughArea)
        {
            setGlobalPosition(caliBase.x, caliBase.y + CIRCLE_DISTANCE + camera_offset);
            moveTo({caliBase.x, caliBase.y + CIRCLE_DISTANCE, caliBase.yaw});
        }
        else
        {
            setGlobalPosition(caliBase.x + CIRCLE_DISTANCE + camera_offset, caliBase.y);
            moveTo({caliBase.x + CIRCLE_DISTANCE, caliBase.y, caliBase.yaw});
        }
    }
    else if (color == 3)
    {
        if (isRoughArea)
        {
            setGlobalPosition(caliBase.x, caliBase.y - CIRCLE_DISTANCE + camera_offset);
            moveTo({caliBase.x, caliBase.y - CIRCLE_DISTANCE, caliBase.yaw});
        }
        else
        {
            setGlobalPosition(caliBase.x - CIRCLE_DISTANCE + camera_offset, caliBase.y);
            moveTo({caliBase.x - CIRCLE_DISTANCE, caliBase.y, caliBase.yaw});
        }
    }
    else
    {
        if (isRoughArea)
        {
            setGlobalPosition(caliBase.x, caliBase.y + camera_offset);
            moveTo({caliBase.x, caliBase.y, caliBase.yaw});
        }
        else
        {
            setGlobalPosition(caliBase.x + camera_offset, caliBase.y);
            moveTo({caliBase.x, caliBase.y, caliBase.yaw});
        }
    }
    //motor->enControl(MOTOR_BROADCAST, true);
}