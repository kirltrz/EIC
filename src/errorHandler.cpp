#include "errorHandler.h"
#include "motion.h"
#include "vision.h"
#include "sensor.h"
#include "config.h"
#include <Arduino.h>

// QR码扫描搜索的相关参数
#define SEARCH_DISTANCE 30.0f   // QR码搜索移动距离，单位mm
#define CIRCLE_SEARCH_DISTANCE 50.0f   // 色环搜索移动距离，单位mm（统一为至少3cm）
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
    
    // 自动判断当前区域：基于位置判断是否在暂存区
    // 暂存区位置: pos[5] = {-1038.0, 1957.0, 0.0f}
    // 判断标准：距离暂存区中心350mm以内视为暂存区
    // 放宽标准是因为小车可以识别到区域两侧的红色和蓝色色环（约30-40cm范围）
    float dist_to_storage = sqrt(pow(originalPos.x - (-1038.0), 2) + pow(originalPos.y - 1957.0, 2));
    bool isStorageArea = (dist_to_storage <= 350.0f);
    
    // 根据区域定义搜索方向：先左右再前后
    float searchOffsets[MAX_SEARCH_ATTEMPTS][2];
    
    if (isStorageArea) {
        // 暂存区：左右是x轴方向，前后是y轴方向
        float offsets[MAX_SEARCH_ATTEMPTS][2] = {
            {-CIRCLE_SEARCH_DISTANCE, 0},          // 向左 (x-)
            {CIRCLE_SEARCH_DISTANCE, 0},           // 向右 (x+)
            {0, CIRCLE_SEARCH_DISTANCE},           // 向前 (y+)
            {0, -CIRCLE_SEARCH_DISTANCE},          // 向后 (y-)
            {-CIRCLE_SEARCH_DISTANCE, CIRCLE_SEARCH_DISTANCE},   // 前左
            {CIRCLE_SEARCH_DISTANCE, CIRCLE_SEARCH_DISTANCE},    // 前右
            {-CIRCLE_SEARCH_DISTANCE, -CIRCLE_SEARCH_DISTANCE},  // 后左
            {CIRCLE_SEARCH_DISTANCE, -CIRCLE_SEARCH_DISTANCE},   // 后右
            {-CIRCLE_SEARCH_DISTANCE * 2, 0},      // 向左远距离
            {CIRCLE_SEARCH_DISTANCE * 2, 0},       // 向右远距离
            {0, CIRCLE_SEARCH_DISTANCE * 2},       // 向前远距离
            {0, -CIRCLE_SEARCH_DISTANCE * 2}       // 向后远距离
        };
        memcpy(searchOffsets, offsets, sizeof(offsets));
    } else {
        // 其他区域：左右是y轴方向，前后是x轴方向
        float offsets[MAX_SEARCH_ATTEMPTS][2] = {
            {0, CIRCLE_SEARCH_DISTANCE},           // 向左 (y+)
            {0, -CIRCLE_SEARCH_DISTANCE},          // 向右 (y-)
            {CIRCLE_SEARCH_DISTANCE, 0},           // 向前 (x+)
            {-CIRCLE_SEARCH_DISTANCE, 0},          // 向后 (x-)
            {CIRCLE_SEARCH_DISTANCE, CIRCLE_SEARCH_DISTANCE},   // 前左
            {CIRCLE_SEARCH_DISTANCE, -CIRCLE_SEARCH_DISTANCE},  // 前右
            {-CIRCLE_SEARCH_DISTANCE, CIRCLE_SEARCH_DISTANCE},  // 后左
            {-CIRCLE_SEARCH_DISTANCE, -CIRCLE_SEARCH_DISTANCE}, // 后右
            {0, CIRCLE_SEARCH_DISTANCE * 2},       // 向左远距离
            {0, -CIRCLE_SEARCH_DISTANCE * 2},      // 向右远距离
            {CIRCLE_SEARCH_DISTANCE * 2, 0},       // 向前远距离
            {-CIRCLE_SEARCH_DISTANCE * 2, 0}       // 向后远距离
        };
        memcpy(searchOffsets, offsets, sizeof(offsets));
    }

    DEBUG_LOG("开始色环搜索，原始位置: x=%.2f, y=%.2f, yaw=%.2f, 区域类型: %s", 
              originalPos.x, originalPos.y, originalPos.rawYaw, 
              isStorageArea ? "暂存区(左右=x轴)" : "其他区域(左右=y轴)");
    
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
    
    // 定义搜索方向：先左右再前后
    float searchOffsets[MAX_SEARCH_ATTEMPTS][2] = {
        {0, SEARCH_DISTANCE},           // 向左
        {0, -SEARCH_DISTANCE},          // 向右
        {SEARCH_DISTANCE, 0},           // 向前
        {-SEARCH_DISTANCE, 0},          // 向后
        {SEARCH_DISTANCE, SEARCH_DISTANCE},   // 前左
        {SEARCH_DISTANCE, -SEARCH_DISTANCE},  // 前右
        {-SEARCH_DISTANCE, SEARCH_DISTANCE},  // 后左
        {-SEARCH_DISTANCE, -SEARCH_DISTANCE}, // 后右
        {0, SEARCH_DISTANCE * 2},       // 向左远距离
        {0, -SEARCH_DISTANCE * 2},      // 向右远距离
        {SEARCH_DISTANCE * 2, 0},       // 向前远距离
        {-SEARCH_DISTANCE * 2, 0}       // 向后远距离
    };

    DEBUG_LOG("开始QR码搜索，原始位置: x=%.2f, y=%.2f, yaw=%.2f (QR码区域: 左右=y轴)", 
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

/**
 * @brief 转盘识别失败时的搜索处理
 * @return 是否成功识别到转盘
 */
static bool searchTurntable() {
    global_position_t originalPos;
    getGlobalPosition(&originalPos);
    
    // 定义搜索方向：先左右再前后
    float searchOffsets[MAX_SEARCH_ATTEMPTS][2] = {
        {0, SEARCH_DISTANCE},           // 向左
        {0, -SEARCH_DISTANCE},          // 向右
        {SEARCH_DISTANCE, 0},           // 向前
        {-SEARCH_DISTANCE, 0},          // 向后
        {SEARCH_DISTANCE, SEARCH_DISTANCE},   // 前左
        {SEARCH_DISTANCE, -SEARCH_DISTANCE},  // 前右
        {-SEARCH_DISTANCE, SEARCH_DISTANCE},  // 后左
        {-SEARCH_DISTANCE, -SEARCH_DISTANCE}, // 后右
        {0, SEARCH_DISTANCE * 2},       // 向左远距离
        {0, -SEARCH_DISTANCE * 2},      // 向右远距离
        {SEARCH_DISTANCE * 2, 0},       // 向前远距离
        {-SEARCH_DISTANCE * 2, 0}       // 向后远距离
    };

    DEBUG_LOG("开始转盘搜索，原始位置: x=%.2f, y=%.2f, yaw=%.2f (转盘区域: 左右=y轴)", 
              originalPos.x, originalPos.y, originalPos.rawYaw);
    
    for (int attempt = 0; attempt < MAX_SEARCH_ATTEMPTS; attempt++) {
        // 计算新的目标位置
        POS targetPos = {
            originalPos.x + searchOffsets[attempt][0],
            originalPos.y + searchOffsets[attempt][1],
            originalPos.rawYaw
        };
        
        DEBUG_LOG("转盘搜索尝试 %d/%d: 移动到 x=%.2f, y=%.2f", 
                  attempt + 1, MAX_SEARCH_ATTEMPTS, targetPos.x, targetPos.y);
        
        // 移动到新位置
        moveTo(targetPos);
        waitArrived();
        
        delay(SEARCH_DELAY);
        
        // 尝试识别转盘
        int x, y;
        if (visionGetTurntable(&x, &y)) {
            DEBUG_LOG("转盘搜索成功！在位置 x=%.2f, y=%.2f，转盘偏移: (%d, %d)", 
                      targetPos.x, targetPos.y, x, y);
            return true;
        }
        
        DEBUG_LOG("转盘第 %d 次尝试失败，继续搜索...", attempt + 1);
    }
    
    // 搜索失败，返回原始位置
    DEBUG_LOG("转盘搜索失败，返回原始位置");
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
        case ERROR_TURNTABLE_RECOGNITION_FAILED:
            {
                DEBUG_LOG("转盘识别失败，开始搜索模式");
                
                // 执行转盘搜索
                bool success = searchTurntable();
                
                if (success) {
                    DEBUG_LOG("转盘搜索处理完成");
                } else {
                    DEBUG_LOG("转盘搜索失败，已尝试所有位置");
                }
            }
            break;
        default:
            DEBUG_LOG("未知错误码: %d", errorCode);
            break;
    }
}
