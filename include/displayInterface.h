#pragma once
#include "lvgl.h"
#include "LGFX.hpp"
#include "ui.h"

void initGUI(void);
void lvglTask(void *pvParameters);

// 夹爪状态UI更新标志
#if DEBUG_ENABLE
extern bool gripperColorUpdateFlag;
#endif
void requestGripperColorUpdate(void);  // 请求更新夹爪颜色状态