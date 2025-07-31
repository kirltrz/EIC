#include "mainSequence.h"
#include "motion.h"
#include "arm.h"
#include "vision.h"
#include "errorHandler.h"
#include "config.h"
#include "sensor.h"

#if !DEBUG_ENABLE
#include "ui.h"
#endif

#define TASK_TIMEOUT 5000
POS pos[] = {
    {-153.0  , 576.0  , 0.0f  }, // 0扫二维码
    {  0.0   , 1420.0 , 0.0f  }, // 1转盘抓取
    {-194.0  , 1000.0 , 80.0f }, // 2离开转盘
    {-1940.0 , 980.0 , 90.0f }, // 3粗加工区
    {-1853.0 , 1824.0 , 10.0f  }, // 4离开粗加工区
    {-1038.0 , 1957.0 , 0.0f  }, // 5暂存区
    {-143.0  , 1857.0 , 0.0f  }, // 6离开暂存区（准备第二轮到转盘）
    {-1021.0 , 187.0  , 0.0f  }, // 7离开暂存区（准备回到启停区）
    {-132.0  , 190.0  , 0.0f  }, // 8准备进入启停区
    {50.0    , -50.0  , 0.0f  }, // 9回到启停区
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

        moveTo({pos[1].x-50.0, pos[1].y, pos[1].yaw}); // 1 先靠近转盘，防止由于误差而超出边界
        //waitNear();
        caliTurntable(); // 根据转盘校准车身
        arm_catchFromTurntable(taskcode[0]);
        P(TASK_FIRST_TURNTABLE);
        moveTo(pos[2]); // 1 前往离开转盘状态
        waitArrived();
        
        moveTo(pos[3]); // 1 前往粗加工区
        waitArrived();
        arm_putToGround(taskcode[0]);
        arm_catchFromGround(taskcode[0]);
        P(TASK_FIRST_ROUGH);
        moveTo(pos[4]); // 1 离开粗加工区
        waitNear();
        
        moveTo(pos[5]); // 1 前往暂存区
        waitArrived();
        arm_putToGround(taskcode[0]);
        P(TASK_FIRST_STORAGE);
        moveTo(pos[6]); // 1 离开暂存区
        waitNear();
        
        moveTo({pos[1].x-100, pos[1].y, pos[1].yaw}); // 1 先靠近转盘，防止由于误差而超出边界
        waitNear();
        caliTurntable(); // 根据转盘校准车身
        arm_catchFromTurntable(taskcode[1]);
        P(TASK_SECOND_TURNTABLE);
        moveTo(pos[2]); // 2 前往离开转盘状态
        waitArrived();
        
        moveTo(pos[3]); // 2 前往粗加工区
        waitArrived();
        arm_putToGround(taskcode[1]);
        arm_catchFromGround(taskcode[1]);
        P(TASK_SECOND_ROUGH);
        moveTo(pos[4]); // 2 离开粗加工区
        waitNear();
        
        moveTo(pos[5]); // 2 前往暂存区
        waitArrived();
        arm_putToMaterial(taskcode[1]);
        P(TASK_SECOND_STORAGE);
        moveTo(pos[7]); // 2 离开暂存区
        waitNear();
        
        moveTo(pos[8]); // 前往启停区
        P(TASK_RETURN_HOME);
        waitNear();
        moveTo(pos[9]); // 回到启停区
        waitArrived();
        
        P(TASK_COMPLETED);
        // 释放信号量，表示主流程已完成
        xSemaphoreGive(xSemaphoreMainsequence);
    }
    
    vTaskDelete(NULL); // 删除当前任务
}

void caliTurntable(void){
    arm_turntableDetect();
    waitArm();
        
    // 根据摄像头返回的与转盘中心的差值逐步靠近转盘
    int x, y;
    const float scale = 0.5f;
    const int threshold = 10; // 阈值，当摄像头返回的数值小于此值时终止校准
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
            current_pos_xy.y += -x * scale * 0.9f+3.0f;
            moveTo(current_pos_xy);
            // 检查是否达到阈值
            if (abs(x) < threshold && abs(y) < threshold) {
                //DEBUG_LOG("转盘校准完成，误差在阈值内: x=%d, y=%d", x, y);
                break;
            }
        } else {
            //DEBUG_LOG("获取转盘位置失败，继续尝试");
            delay(100);
        }
    }
    
    if (millis() - start_time >= timeout_ms) {
        //DEBUG_LOG("转盘校准超时");
    }
    
    // 设置全局定位xy数据为pos[1]中的数据
    setGlobalPosition(pos[1].x, pos[1].y);
    moveTo(pos[1]);
}