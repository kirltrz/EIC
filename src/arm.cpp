#include "arm.h"

FSUS_Protocol protocol(&Serial1, 115200);
FSUS_Servo servo0(0, &protocol); // 云台舵机
FSUS_Servo servo1(1, &protocol); // 一级关节舵机
FSUS_Servo servo2(2, &protocol); // 二级关节舵机
FSUS_Servo servo3(3, &protocol); // 三级关节舵机
FSUS_Servo servo4(4, &protocol); // 夹爪舵机

struct armPos
{
    int x;
    int y;
    int z;
};
const armPos fold = {0, 0, 0};       // 折叠状态
const armPos ttDetect = {0, 0, 0};   // 转盘检测位置
const armPos redPlate = {0, 0, 0};   // 红色物料托盘位置
const armPos bluePlate = {0, 0, 0};  // 蓝色物料托盘位置
const armPos greenPlate = {0, 0, 0}; // 绿色物料托盘位置
const armPos redCircleBase = {0, 0, 0}; // 红色色环基础位置（未视觉纠偏的位置）
const armPos blueCircleBase = {0, 0, 0}; // 蓝色色环基础位置（未视觉纠偏的位置）
const armPos greenCircleBase = {0, 0, 0}; // 绿色色环基础位置（未视觉纠偏的位置）

struct
{ // 当前车身位置下的色环位置
    struct
    {
        int x;
        int y;
    } redCircle, blueCircle, greenCircle;
} circlePosition;

void initArm(void)
{
    /*初始化机械臂*/
    protocol.init(); // 初始化协议
    servo0.init();   // 初始化云台舵机
    servo1.init();   // 初始化一级关节舵机
    servo2.init();   // 初始化二级关节舵机
    servo3.init();   // 初始化三级关节舵机
    servo4.init();   // 初始化夹爪舵机
}

void moveArm(int x, int y, int z)
{
    /*移动机械臂*/
    // 机械臂参数定义
    const float L1 = 100.0; // 一级臂长度(mm)
    const float L2 = 100.0; // 二级臂长度(mm)
    const float L3 = 100.0; // 三级臂长度(mm)

    // 逆运动学计算
    float r = sqrt(x * x + y * y); // 水平距离
    float R = sqrt(r * r + z * z); // 空间距离

    // 检查是否超出工作范围
    if (R > (L1 + L2 + L3) || R < abs(L1 - L2 - L3))
    {
        // 目标点超出工作范围
        return;
    }

    // 计算关节角度
    float theta0 = atan2(y, x) * 180.0 / PI; // 云台舵机角度(度)

    // 使用余弦定理计算关节角度
    float alpha = atan2(z, r) * 180.0 / PI;
    float beta = acos((L1 * L1 + R * R - L2 * L2 - L3 * L3 - 2 * L2 * L3) / (2 * L1 * R)) * 180.0 / PI;
    float theta1 = alpha + beta; // 一级关节舵机角度(度)

    float gamma = acos((L1 * L1 + L2 * L2 - R * R + L3 * L3 + 2 * L2 * L3) / (2 * L1 * L2)) * 180.0 / PI;
    float theta2 = 180.0 - gamma; // 二级关节舵机角度(度)

    float delta = acos((L2 * L2 + L3 * L3 - L1 * L1 - R * R + 2 * L1 * R) / (2 * L2 * L3)) * 180.0 / PI;
    float theta3 = 180.0 - delta; // 三级关节舵机角度(度)

    // 设置舵机角度
    servo0.setAngle(theta0, 1000); // 云台舵机
    servo1.setAngle(theta1, 1000); // 一级关节舵机
    servo2.setAngle(theta2, 1000); // 二级关节舵机
    servo3.setAngle(theta3, 1000); // 三级关节舵机

    // 等待舵机运动完成
    servo0.wait();
    servo1.wait();
    servo2.wait();
    servo3.wait();
}

void catchFromTurntable(int taskcode[3])
{
    /*从转盘抓取*/
    const int turntableHeight = 80; // 转盘高度
}

void putToGround(int taskcode[3])
{
    /*放置到地面*/
}

void catchFromGround(int taskcode[3])
{
    /*从地面抓取*/
}

void putToMaterial(int taskcode[3])
{
    /*码垛*/
}
