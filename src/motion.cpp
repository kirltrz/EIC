#include "motion.h"

void initMotor(void)
{
    /*初始化电机*/
}
void moveTo(POS pos, float speed, int acc, int dec)
{
    /*移动到目标点，仅用于传入目标点，具体逻辑请自行创建函数实现*/
    
}
void moveTask(void*pvParameters)
{
    /*移动任务,不断向电机发送控制指令*/
}
bool arrived(void)
{
    /*判断是否到达目标点*/
    // 1. 获取当前点的位置
    // 2. 判断当前点与目标点的距离
    // 3. 如果距离小于一定值，则返回true，否则返回false
    return false;
}