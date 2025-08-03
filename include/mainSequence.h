#pragma once
#include"taskManager.h"
#include"motion.h"
#include"devMonitor.h"

// 校准模式枚举
enum CaliMode {
    CALI_ROUGH = 0,      // 粗定位（机械臂标准位）
    CALI_PRECISE = 1,    // 精确定位（机械臂低位）
    CALI_STACKING = 2    // 码垛定位（粗定位+机械臂高位）
};

extern struct POS pos[10];
extern struct POS que[15];
extern int taskcode[2][3];

void caliTurntable(void);
void caliCircle(POS caliBase, CaliMode mode);
void startMainSequence(void);
void mainSequenceTask(void *);
void updateTaskcodeDisplay(void);