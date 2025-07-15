#include "errorHandler.h"
#include "motion.h"
#include "vision.h"
#include "sensor.h"
#include "config.h"
#include <Arduino.h>

// QR码扫描搜索的相关参数
#define SEARCH_DISTANCE 30.0f   // QR码搜索移动距离，单位mm
#define CIRCLE_SEARCH_DISTANCE 15.0f   // 色环搜索移动距离，单位mm
#define MAX_SEARCH_ATTEMPTS 12  // 最大搜索尝试次数
#define SEARCH_DELAY 500       // 每次搜索间隔时间，单位ms

// 定义全局变量
int circleX = 0, circleY = 0;
bool circleFound = false;

/**
 * @brief 生成随机任务码（1、2、3的排列组合）
 */
static void generateRandomTaskcodes() {
    // 所有可能的1、2、3排列组合
    int permutations[6][3] = {
        {1, 2, 3},
        {1, 3, 2},
        {2, 1, 3},
        {2, 3, 1},
        {3, 1, 2},
        {3, 2, 1}
    };
    
    // 初始化随机数种子（使用当前时间）
    randomSeed(millis());
    
    // 随机选择第一个任务码
    int index1 = random(0, 6);
    taskcode[0][0] = permutations[index1][0];
    taskcode[0][1] = permutations[index1][1];
    taskcode[0][2] = permutations[index1][2];
    
    // 随机选择第二个任务码（完全随机，可能与第一个相同）
    int index2 = random(0, 6);
    taskcode[1][0] = permutations[index2][0];
    taskcode[1][1] = permutations[index2][1];
    taskcode[1][2] = permutations[index2][2];
    
    DEBUG_LOG("随机生成任务码: %d%d%d+%d%d%d", 
              taskcode[0][0], taskcode[0][1], taskcode[0][2],
              taskcode[1][0], taskcode[1][1], taskcode[1][2]);
}

/**
 * @brief 色环识别失败时的搜索处理
 * @return 是否成功识别到色环
 */
static bool searchCircle() {
    global_position_t originalPos;
    getGlobalPosition(&originalPos);
    
    // 定义搜索方向，使用较小的搜索距离
    float searchOffsets[MAX_SEARCH_ATTEMPTS][2] = {
        {CIRCLE_SEARCH_DISTANCE, 0},           // 向前
        {0, -CIRCLE_SEARCH_DISTANCE},          // 向右
        {-CIRCLE_SEARCH_DISTANCE, 0},          // 向后
        {0, CIRCLE_SEARCH_DISTANCE},           // 向左
        {CIRCLE_SEARCH_DISTANCE, -CIRCLE_SEARCH_DISTANCE},  // 前右
        {-CIRCLE_SEARCH_DISTANCE, -CIRCLE_SEARCH_DISTANCE}, // 后右
        {-CIRCLE_SEARCH_DISTANCE, CIRCLE_SEARCH_DISTANCE},  // 后左
        {CIRCLE_SEARCH_DISTANCE, CIRCLE_SEARCH_DISTANCE},   // 前左
        {CIRCLE_SEARCH_DISTANCE * 2, 0},       // 向前远距离
        {-CIRCLE_SEARCH_DISTANCE * 2, 0},      // 向后远距离
        {0, CIRCLE_SEARCH_DISTANCE * 2},       // 向左远距离
        {0, -CIRCLE_SEARCH_DISTANCE * 2}       // 向右远距离
    };

    DEBUG_LOG("开始色环搜索，原始位置: x=%.2f, y=%.2f, yaw=%.2f", 
              originalPos.x, originalPos.y, originalPos.rawYaw);
    
    for (int attempt = 0; attempt < MAX_SEARCH_ATTEMPTS; attempt++) {
        // 计算新的目标位置
        POS targetPos = {
            originalPos.x + searchOffsets[attempt][0],
            originalPos.y + searchOffsets[attempt][1],
            originalPos.rawYaw
        };
        
        DEBUG_LOG("色环搜索尝试 %d/%d: 移动到 x=%.2f, y=%.2f", 
                  attempt + 1, MAX_SEARCH_ATTEMPTS, targetPos.x, targetPos.y);
        
        // 移动到新位置
        moveTo(targetPos);
        waitArrived();
        
        delay(SEARCH_DELAY);
        
        // 尝试识别色环
        if (visionGetCircle(&circleX, &circleY)) {
            DEBUG_LOG("色环搜索成功！在位置 x=%.2f, y=%.2f，色环坐标: (%d, %d)", 
                      targetPos.x, targetPos.y, circleX, circleY);
            circleFound = true;
            return true;
        }
        
        DEBUG_LOG("色环第 %d 次尝试失败，继续搜索...", attempt + 1);
    }
    
    // 搜索失败，返回原始位置
    DEBUG_LOG("色环搜索失败，返回原始位置");
    POS returnPos = {originalPos.x, originalPos.y, originalPos.rawYaw};
    moveTo(returnPos);
    waitArrived();
    
    circleFound = false;
    return false;
}

/**
 * @brief QR码识别失败时的搜索处理
 * @return 是否成功扫描到QR码
 */
static bool searchQRCode() {
    global_position_t originalPos;
    getGlobalPosition(&originalPos);
    
    // 定义搜索方向：前、右、后、左、前右、后右、后左、前左、前方远距离、后方远距离、左侧远距离、右侧远距离
    float searchOffsets[MAX_SEARCH_ATTEMPTS][2] = {
        {SEARCH_DISTANCE, 0},           // 向前
        {0, -SEARCH_DISTANCE},          // 向右
        {-SEARCH_DISTANCE, 0},          // 向后
        {0, SEARCH_DISTANCE},           // 向左
        {SEARCH_DISTANCE, -SEARCH_DISTANCE},  // 前右
        {-SEARCH_DISTANCE, -SEARCH_DISTANCE}, // 后右
        {-SEARCH_DISTANCE, SEARCH_DISTANCE},  // 后左
        {SEARCH_DISTANCE, SEARCH_DISTANCE},   // 前左
        {SEARCH_DISTANCE * 2, 0},       // 向前远距离
        {-SEARCH_DISTANCE * 2, 0},      // 向后远距离
        {0, SEARCH_DISTANCE * 2},       // 向左远距离
        {0, -SEARCH_DISTANCE * 2}       // 向右远距离
    };

    DEBUG_LOG("开始QR码搜索，原始位置: x=%.2f, y=%.2f, yaw=%.2f", 
              originalPos.x, originalPos.y, originalPos.rawYaw);
    
    for (int attempt = 0; attempt < MAX_SEARCH_ATTEMPTS; attempt++) {
        // 计算新的目标位置
        POS targetPos = {
            originalPos.x + searchOffsets[attempt][0],
            originalPos.y + searchOffsets[attempt][1],
            originalPos.rawYaw
        };
        
        DEBUG_LOG("尝试 %d/%d: 移动到 x=%.2f, y=%.2f", 
                  attempt + 1, MAX_SEARCH_ATTEMPTS, targetPos.x, targetPos.y);
        
        // 移动到新位置
        moveTo(targetPos);
        waitArrived();  // 等待到达目标位置
        
        delay(SEARCH_DELAY);  // 等待稳定
        
        // 尝试扫描QR码，使用全局taskcode变量
        if (visionScanQRcode(taskcode[0], taskcode[1])) {
            DEBUG_LOG("QR码搜索成功！在位置 x=%.2f, y=%.2f", targetPos.x, targetPos.y);
            DEBUG_LOG("获取的任务码: %d%d%d+%d%d%d", 
                      taskcode[0][0], taskcode[0][1], taskcode[0][2],
                      taskcode[1][0], taskcode[1][1], taskcode[1][2]);
            return true;
        }
        
        DEBUG_LOG("第 %d 次尝试失败，继续搜索...", attempt + 1);
    }
    
    // 搜索失败，返回原始位置
    DEBUG_LOG("QR码搜索失败，返回原始位置");
    POS returnPos = {originalPos.x, originalPos.y, originalPos.rawYaw};
    moveTo(returnPos);
    waitArrived();
    
    return false;
}

//错误处理函数 - 唯一入口
void errorHandle(int errorCode){
    switch(errorCode){
        case ERROR_QRCODE_RECOGNITION_FAILED:
            {
                DEBUG_LOG("QR码识别失败，开始搜索模式");
                
                // 执行QR码搜索
                bool success = searchQRCode();
                
                if (success) {
                    DEBUG_LOG("QR码搜索处理完成");
                } else {
                    DEBUG_LOG("QR码搜索失败，已尝试所有位置");
                    // 随机生成任务码
                    generateRandomTaskcodes();
                }
            }
            break;
        case ERROR_CIRCLE_RECOGNITION_FAILED:
            {
                DEBUG_LOG("色环识别失败，开始搜索模式");
                
                // 执行色环搜索
                bool success = searchCircle();
                
                if (success) {
                    DEBUG_LOG("色环搜索处理完成，坐标: (%d, %d)", circleX, circleY);
                } else {
                    DEBUG_LOG("色环搜索失败，跳过该步骤");
                    circleFound = false;
                }
            }
            break;
        case ERROR_MATERIAL_RECOGNITION_FAILED:
            DEBUG_LOG("物料识别失败，暂时不处理");
            break;
        default:
            DEBUG_LOG("未知错误码: %d", errorCode);
            break;
    }
}
