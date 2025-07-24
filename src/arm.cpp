#include "arm.h"
#include "config.h"
#include "vision.h"
#include "taskManager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include "errorHandler.h"
#if DEBUG_ENABLE
#include "displayInterface.h" // 用于请求UI更新
#endif
FSUS_Protocol protocol(&SERVO_SERIAL, SERIAL_BAUDRATE);
FSUS_Servo servo0(0, &protocol); // 云台舵机
FSUS_Servo servo1(1, &protocol); // 一级关节舵机
FSUS_Servo servo2(2, &protocol); // 二级关节舵机
FSUS_Servo servo3(3, &protocol); // 三级关节舵机
FSUS_Servo servo4(4, &protocol); // 夹爪舵机

// 夹爪位置监测相关全局变量
bool gripperHolding = false;                  // 夹持状态标志，false表示未夹持，true表示已夹持
float currentGripperAngle = 0.0f;             // 当前夹爪角度，单位度
TaskHandle_t gripperMonitorTaskHandle = NULL; // 夹爪监测任务句柄
SemaphoreHandle_t gripperMutex = NULL;        // 夹爪状态互斥锁

const float scale = 0.1f;

struct armPos
{
    int x;
    int y;
    int z;
};
const armPos fold = {35, 0, 99};          // 折叠状态
const armPos ttDetect = {240, 0, 240};    // 转盘检测位置
const armPos rPlate = {65, -62, 90};      // 红色物料托盘位置
const armPos bPlate = {-67, 63, 88};      // 蓝色物料托盘位置
const armPos gPlate = {61, 62, 91};       // 绿色物料托盘位置
const int plateAngle[3] = {136, 46, -46}; // 托盘角度
const armPos roverthecircle = {0, 0, 0};  // 红色物料空中位置
const armPos boverthecircle = {0, 0, 0};  // 蓝色物料空中位置
const armPos goverthecircle = {0, 0, 0};  // 绿色物料空中位置
// const armPos gdDetect = {0, 0, 0};    // 地面检测位置
const armPos rCircleBase = {145, 220, 0};  // 红色色环基础位置（未视觉纠偏的位置）
const armPos bCircleBase = {-155, 200, 0}; // 蓝色色环基础位置（未视觉纠偏的位置）
const armPos gCircleBase = {-6, 210, 0};   // 绿色色环基础位置（未视觉纠偏的位置）

armPos platePos[3] = {rPlate, gPlate, bPlate};
armPos circlePos[3] = {rCircleBase, gCircleBase, bCircleBase};

armPos keyPos[12][5] = {
    {// 红色 从托盘到色环关键点
     {67, -64, 120},
     {75, 0, 161},
     {82, 60, 200},
     {107, 144, 158},
     {117, 170, 110}},
    {// 绿色 从托盘到色环关键点
     {64, 66, 136},
     {50, 100, 125},
     {20, 145, 106},
     {0, 166, 80},
     {0, 187, 42}},
    {// 蓝色 从托盘到色环关键点
     {-71, 66, 120},
     {-100, 100, 120},
     {-125, 150, 95},
     {-135, 170, 65},
     {-145, 178, 60}},
    {// 红色 从色环到托盘关键点
     {117, 170, 110},
     {107, 144, 158},
     {82, 60, 200},
     {75, 0, 161},
     {67, -64, 120}},
    {// 绿色 从色环到托盘关键点
     {0, 187, 42},
     {0, 166, 80},
     {20, 145, 106},
     {50, 100, 125},
     {64, 66, 136}},
    {// 蓝色 从色环到托盘关键点
     {-145, 178, 60},
     {-135, 170, 65},
     {-125, 150, 95},
     {-100, 100, 120},
     {-71, 66, 120}},
    {// 红色 从托盘到色环上的物料的关键点
     {67, -64, 120},
     {75, 0, 161},
     {82, 60, 200},
     {107, 144, 158},
     {117, 170, 110}},
    {// 绿色 从托盘到色环上的物料的关键点
     {64, 66, 136},
     {50, 100, 125},
     {20, 145, 115},
     {0, 166, 100},
     {0, 187, 90}},
    {// 蓝色 从托盘到色环上的物料的关键点
     {-71, 66, 120},
     {-100, 100, 120},
     {-125, 150, 95},
     {-135, 170, 85},
     {-145, 178, 80}}};

const int overPlateHeight = 50;  // 机器爪抓取物料之前高于物料的高度
const int overCircleHeight = 50; // 机器爪放置物料后高于色环的高度

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
    SERVO_SERIAL.begin(115200, SERIAL_8N1, PIN_SERVO_RX, PIN_SERVO_TX);
    /*初始化机械臂*/
    protocol.init(); // 初始化协议
    servo0.init();   // 初始化云台舵机
    servo1.init();   // 初始化一级关节舵机
    servo2.init();   // 初始化二级关节舵机
    servo3.init();   // 初始化三级关节舵机
    servo4.init();   // 初始化夹爪舵机
    /*舵机角度限幅 */
    servo0.setAngleRange(BASIS_ARM_ANGLE_MIN, BASIS_ARM_ANGLE_MAX);        // 云台舵机
    servo1.setAngleRange(FIRST_ARM_ANGLE_MIN, FIRST_ARM_ANGLE_MAX);        // 一级关节舵机
    servo2.setAngleRange(SECOND_ARM_ANGLE_MIN, SECOND_ARM_ANGLE_MAX);      // 二级关节舵机
    servo3.setAngleRange(THIRD_ARM_ANGLE_MIN, THIRD_ARM_ANGLE_MAX);        // 三级关节舵机
    servo4.setAngleRange(ARM_GRIPPER_CLOSE_ANGLE, ARM_GRIPPER_OPEN_ANGLE); // 夹爪舵机
    stopArm(2000);
    DEBUG_LOG("初始化机械臂完成");
}
void setOriginPoint(void)
{
    servo0.SetOriginPoint();
    servo1.SetOriginPoint();
    servo2.SetOriginPoint();
    servo3.SetOriginPoint();
    // servo4.SetOriginPoint();//夹爪舵机一般不需要归零
}
void stopArm(uint16_t power)
{
    servo0.setDamping(power);
    servo1.setDamping(power);
    servo2.setDamping(power);
    servo3.setDamping(power);
    servo4.setDamping(power);
}
/*逆解算 根据下x,y,z坐标解算出具体关节的舵机角度*/
bool armCalculate_inverse(float x, float y, float z, float *out_arm_degree)
{
    float theta0;
    float w;
    float shortSide;        // 短边长度
    float hypotenuse;       // 斜边长度
    float first_arm_degree, // 大臂角度
        first_arm_rad_1,    // 大臂弧度计算中间值1
        first_arm_rad_2,
        second_arm_degree,  // 小臂角度
        _second_arm_degree, // 小臂角度计算中间值
        third_arm_degree;   // 大臂弧度计算中间值2

    /*通过xy坐标计算云台角度*/
    theta0 = atan2(x, y) * RAD_TO_DEG;

    // 考虑物料偏移
    x = x - (ARM_MATERIAL_OFFSET_X * cos(theta0 * DEG_TO_RAD));
    y = y + (ARM_MATERIAL_OFFSET_X * sin(theta0 * DEG_TO_RAD));

    /*计算w轴长度*/
    w = sqrt(x * x + y * y);

    // 考虑爪子长度高度,修正坐标系
    w = w - ARM_FIRST_JOINT_OFFSET_W - ARM_MATERIAL_OFFSET_W;
    z = z + ARM_THIRD_LENGTH + ARM_MATERIAL_HEIGHT;
    if (w < 0)
        w = 0;

    // 计算斜边
    shortSide = z - ARM_FIRST_JOINT_HEIGHT;
    hypotenuse = sqrt(w * w + shortSide * shortSide);

    // 检查三角形是否成立
    if (hypotenuse > (ARM_FIRST_LENGTH + ARM_SECOND_LENGTH) ||
        hypotenuse < fabs(ARM_FIRST_LENGTH - ARM_SECOND_LENGTH))
    {
        DEBUG_LOG("\n目标位置无法到达");
        return false;
    }

    // 计算大臂角度
    first_arm_rad_1 =
        acos(((ARM_FIRST_LENGTH * ARM_FIRST_LENGTH) - (ARM_SECOND_LENGTH * ARM_SECOND_LENGTH) + hypotenuse * hypotenuse) /
             (2.0f * ARM_FIRST_LENGTH * hypotenuse));

    first_arm_rad_2 = atan2(abs(shortSide), w);

    // 计算大臂角度
    if (z > ARM_FIRST_JOINT_HEIGHT)
        first_arm_degree = 90.0f - (first_arm_rad_1 + first_arm_rad_2) * RAD_TO_DEG; // 大臂角度(度)
    else
        first_arm_degree = 90.0f - (first_arm_rad_1 - first_arm_rad_2) * RAD_TO_DEG;

    // 计算小臂角度
    _second_arm_degree =
        acos(((ARM_SECOND_LENGTH * ARM_SECOND_LENGTH) + (ARM_FIRST_LENGTH * ARM_FIRST_LENGTH) - hypotenuse * hypotenuse) /
             (2.0f * ARM_FIRST_LENGTH * ARM_SECOND_LENGTH)) *
        RAD_TO_DEG; // 小臂角度(度)

    second_arm_degree = 90.0f - _second_arm_degree; //

    third_arm_degree = -(90.0f - (_second_arm_degree - first_arm_degree));

    out_arm_degree[0] = theta0;            // 云台舵机角度(度)
    out_arm_degree[1] = first_arm_degree;  // 大臂角度
    out_arm_degree[2] = second_arm_degree; // 小臂角度(度)
    out_arm_degree[3] = third_arm_degree;  // 三级关节舵机角度(度)
    return true;
}
/**
 * 根据大小臂角度计算末端执行器的位置坐标
 *@brief 机械臂运动学正解算
 *@param first_arm_degree: 大臂角度,竖直方向为0,向前为正角度
 *@param second_arm_degree: 小臂角度,水平方向为0,向上为正角度
 *@param theta0   云台舵机角度(度)
 *@param out_arm_location_xyz[0]: 输出坐标x
 *@param out_arm_location_xyz[1]: 输出坐标y
 *@param out_arm_location_xyz[2]: 输出坐标z
 */
void armCalculate_forward(float theta0, float first_arm_degree, float second_arm_degree, float *out_arm_location_xyz)
{
    float x, x1, x2;
    float y, y1, y2;
    float w, w1, w2; // w为机械臂坐标系下的y轴
    float z, z1, z2;

    w1 = ARM_FIRST_LENGTH * sin(first_arm_degree * DEG_TO_RAD); // 大臂y轴增量
    w2 = ARM_SECOND_LENGTH * cos((first_arm_degree + second_arm_degree) * DEG_TO_RAD);
    w = w1 + w2 + ARM_MATERIAL_OFFSET_W + ARM_FIRST_JOINT_OFFSET_W;

    y1 = w * cos(theta0 * DEG_TO_RAD);                     // 将w轴增量转换到机械臂中心坐标系下的y轴增量
    y2 = ARM_MATERIAL_OFFSET_X * sin(theta0 * DEG_TO_RAD); // 将机械臂坐标系下的物料x轴偏移转换到机械臂中心坐标系下
    y = y1 - y2;

    x1 = w * sin(theta0 * DEG_TO_RAD);                     // 使用机械臂坐标系下的y轴增量计算小车坐标系下的x轴增量
    x2 = ARM_MATERIAL_OFFSET_X * cos(theta0 * DEG_TO_RAD); // 将机械臂坐标系下的物料x轴偏移转换到小车坐标系下
    x = x1 + x2;

    z1 = ARM_FIRST_LENGTH * cos(first_arm_degree * DEG_TO_RAD); // 大臂z轴增量
    z2 = ARM_SECOND_LENGTH * sin((first_arm_degree + second_arm_degree) * DEG_TO_RAD);
    z = z1 - z2 - ARM_THIRD_LENGTH + ARM_FIRST_JOINT_HEIGHT - ARM_MATERIAL_HEIGHT; // 末端执行器z坐标

    out_arm_location_xyz[0] = x;
    out_arm_location_xyz[1] = y;
    out_arm_location_xyz[2] = z;
}
/**
 *@brief 机械大小臂角度控制
 *@param first_arm_degree: 大臂相对角度
 *@param second_arm_degree: 小臂相对角度
 *@return 无
 */
void armSet_position(float theta0, float first_arm_degree, float second_arm_degree, float third_arm_degree, uint16_t interval, uint16_t acc, uint16_t dec)
{
    // 设置舵机角度 这里启用舵机加减速 但加速减速参数待调
    servo0.setAngle(theta0, interval, acc, dec); // 云台舵机
    delay(3);
    servo1.setAngle(first_arm_degree - 1.0f, interval, acc, dec); // 一级关节舵机
    delay(3);
    servo2.setAngle(second_arm_degree - 0.5f, interval, acc, dec); // 二级关节舵机
    delay(3);
    servo3.setAngle(third_arm_degree, interval, acc, dec); // 三级关节舵机
}
/**
 *@brief 机械臂xyz平面位置控制
 *@param x: x坐标,单位mm
 *@param y: y坐标,单位mm
 *@param z: z坐标,单位mm
 *@return void
 */
void armControl_xyz(float x, float y, float z, uint16_t interval, uint16_t acc, uint16_t dec, bool needDelay)
{
    float xyz_to_angle[4];

    if (armCalculate_inverse(x, y, z, xyz_to_angle)) // 逆解算
    {
        // DEBUG_LOG("机械臂xyz坐标: x=%f, y=%f, z=%f", x, y, z);
        armSet_position(xyz_to_angle[0], xyz_to_angle[1], xyz_to_angle[2], xyz_to_angle[3], interval, acc, dec);
        if (needDelay)
            delay(interval);
        if (abs(servo0.queryAngle() - xyz_to_angle[0]) > 10 ||
            abs(servo1.queryAngle() - xyz_to_angle[1]) > 10 ||
            abs(servo2.queryAngle() - xyz_to_angle[2]) > 10 ||
            abs(servo3.queryAngle() - xyz_to_angle[3]) > 10)
        {
            DEBUG_LOG("机械臂角度误差过大，重新发送控制指令");
            armSet_position(xyz_to_angle[0], xyz_to_angle[1], xyz_to_angle[2], xyz_to_angle[3], interval, acc, dec);
            if (needDelay)
            delay(interval);
        }
    }
}

void arm_ScanQRcode()
{
    /*扫描二维码，与控制xyz不同，机械臂前端需抬起使摄像头朝向二维码，无需处理视觉部分*/
    armSet_position(90.0, -50.0, 50.0, -64.0, 500, 100, 100);
    arm_setClaw(1);
}
void arm_setClaw(bool open)
{
    if (open)
    {
        servo4.setAngle(ARM_GRIPPER_OPEN_ANGLE, 100);
    }
    else
    {
        servo4.setAngle(ARM_GRIPPER_CLOSE_ANGLE, 100);
    }
}
void waitArm(void)
{
    servo0.wait();
    servo1.wait();
    servo2.wait();
    servo3.wait();
    servo4.wait();
}

// 任务参数结构体
typedef struct
{
    int taskcode[3];
} ArmTaskParams;

void arm_catchFromTurntable(int taskcode[3]) // 将物料从转盘抓取到托盘
{
    // 分配任务参数内存
    ArmTaskParams *params = (ArmTaskParams *)malloc(sizeof(ArmTaskParams));
    if (params == NULL)
    {
        DEBUG_LOG("内存分配失败");
        return;
    }

    // 复制参数
    for (int i = 0; i < 3; i++)
    {
        params->taskcode[i] = taskcode[i];
    }

    // 使用lambda表达式创建任务
    auto taskFunction = [](void *parameter) -> void
    {
        ArmTaskParams *params = (ArmTaskParams *)parameter;
        int *taskcode = params->taskcode;

        /*从转盘抓取，需要通过视觉识别抓取物料，因为转盘不断转动，故需要等待物料停止再抓取或者实时跟踪*/
        int x, y;
        int arm_x, arm_y;
        int startTime = 0;
        const int traceHeightOffset = 10;
        const int turntableHeight = 80; // 转盘高度
        float y_offset;
        float x_offset;
        float servo0_angle;
        for (int i = 0; i < 3; i++) // 循环3次，分别抓取3种颜色物料
        {
            armControl_xyz(ttDetect.x, ttDetect.y, ttDetect.z, 1000, 100, 100, false); // 停在转盘中心，xyz的单位为mm
            arm_setClaw(1);                                                            // 张开夹爪以便检测物料
            startTime = millis();
            // 以上一次的xy为基础叠加摄像头数据进行跟踪，若进入死区持续一定时间或超时则退出循环执行抓取
            int follow_x = ttDetect.x;
            int follow_y = ttDetect.y;
            const int deadzone = 20;        // 死区阈值，单位与x/y一致
            const int deadzone_time = 1000; // 死区持续时间ms
            int deadzone_start = 0;
            bool in_deadzone = false;
            while (1)
            {
                if (millis() - startTime > VISION_GET_MATERIAL_TIMEOUT)
                {
                    DEBUG_LOG("第%d种颜色物料跟踪超时", taskcode[i]);
                    errorHandle(ERROR_MATERIAL_RECOGNITION_FAILED);
                    break;
                }

                // 直接使用vision模块内部的重试逻辑
                if (!visionGetMaterial(taskcode[i], &x, &y))
                {
                    // vision模块获取失败，继续循环
                    continue;
                }

                // 获取当前云台角度
                float current_angle = servo0.queryAngle();
                float global_x, global_y;

                // 转换摄像头局部坐标到全局坐标
                localToGlobalCoordinates(x, y, current_angle, &global_x, &global_y);

                // 使用转换后的坐标更新跟踪位置
                follow_x += global_x * scale;
                follow_y += global_y * scale;

                armControl_xyz(follow_x, follow_y, ttDetect.z - traceHeightOffset, 1000, 100, 100, true);

                // 判断是否进入死区
                if (abs(x) < deadzone && abs(y) < deadzone)
                {
                    if (!in_deadzone)
                    {
                        in_deadzone = true;
                        deadzone_start = millis();
                    }
                    else if (millis() - deadzone_start > deadzone_time)
                    {
                        // 死区持续足够时间，退出循环
                        DEBUG_LOG("第%d种颜色物料已进入死区，准备抓取", taskcode[i]);
                        break;
                    }
                }
                else
                {
                    in_deadzone = false;
                }
            }
            /*摄像头和夹爪中心点不同，通过三角函数计算偏移量以使夹爪中心点对准物料中心点*/
            servo0_angle = servo0.queryAngle();
            sincosf(servo0_angle * DEG_TO_RAD, &x_offset, &y_offset);
            x_offset = x_offset * (-ARM_MATERIAL_OFFSET_W);
            y_offset = y_offset * (-ARM_MATERIAL_OFFSET_W);
            /*移动到物料上方并抓取*/
            armControl_xyz(ttDetect.x + x * scale + x_offset, ttDetect.y + y * scale + y_offset, turntableHeight, 1000, 100, 100, true);
            arm_setClaw(0); // 闭合夹爪

            arm_x = ttDetect.x + x * scale + x_offset;
            arm_y = ttDetect.y + y * scale + y_offset;
            if (arm_x < 280)
            {
                armControl_xyz(arm_x, arm_y, turntableHeight + 100, 1000, 100, 100, true);
                armControl_xyz(85, 0, 180, 1000, 100, 100, true);
                servo0.setAngle(plateAngle[taskcode[i] - 1], 500, 250, 250);
                delay(500);
                armControl_xyz(platePos[taskcode[i] - 1].x, platePos[taskcode[i] - 1].y, platePos[taskcode[i] - 1].z, 1000, 100, 100, true);
                arm_setClaw(1);
                delay(500);
                armControl_xyz(platePos[taskcode[i] - 1].x, platePos[taskcode[i] - 1].y, platePos[taskcode[i] - 1].z + 100, 1000, 100, 100, true);
                servo0.setAngle(90.0f, 500, 250, 250);
                delay(500);
            }
            else
            {
                armControl_xyz(ttDetect.x, ttDetect.y, turntableHeight, 1000, 100, 100, true);
                armControl_xyz(arm_x, arm_y, turntableHeight + 100, 1000, 100, 100, true);
                armControl_xyz(85, 0, 180, 1000, 100, 100, true);
                servo0.setAngle(plateAngle[taskcode[i] - 1], 500, 250, 250);
                delay(500);
                armControl_xyz(platePos[taskcode[i] - 1].x, platePos[taskcode[i] - 1].y, platePos[taskcode[i] - 1].z, 1000, 100, 100, true);
                arm_setClaw(1);
                delay(500);
                armControl_xyz(platePos[taskcode[i] - 1].x, platePos[taskcode[i] - 1].y, platePos[taskcode[i] - 1].z + 100, 1000, 100, 100, true);
                servo0.setAngle(90.0f, 500, 250, 250);
                delay(500);
            }
        } // 关闭 for 循环

        armResumeMainSequence();
        // 释放参数内存并删除任务
        free(params);
        vTaskDelete(NULL);
    };

    // 创建FreeRTOS任务
    BaseType_t result = xTaskCreate(
        taskFunction,            // lambda表达式作为任务函数
        "ArmCatchFromTurntable", // 任务名称
        4096,                    // 堆栈大小
        params,                  // 任务参数
        1,                       // 任务优先级
        NULL                     // 任务句柄
    );

    if (result != pdPASS)
    {
        DEBUG_LOG("创建arm_catchFromTurntable任务失败");
        free(params);
    }
    armPauseMainSequence();
}

void arm_putToGround(int taskcode[3]) // 将第一次的物料放置到地面的色环
{
    // 分配任务参数内存
    ArmTaskParams *params = (ArmTaskParams *)malloc(sizeof(ArmTaskParams));
    if (params == NULL)
    {
        DEBUG_LOG("内存分配失败");
        return;
    }

    // 复制参数
    for (int i = 0; i < 3; i++)
    {
        params->taskcode[i] = taskcode[i];
    }

    // 使用lambda表达式创建任务
    auto taskFunction = [](void *parameter) -> void
    {
        ArmTaskParams *params = (ArmTaskParams *)parameter;
        int *taskcode = params->taskcode;

        /*放置到地面，需要通过视觉识别放置位置，然后在预先设置的位置基础上叠加偏移量进行放置*/
        for (int i = 0; i < 3; i++)
        {
            arm_setClaw(1);
            armControl_xyz(platePos[taskcode[i] - 1].x, platePos[taskcode[i] - 1].y, platePos[taskcode[i] - 1].z + overPlateHeight, 400, 200, 200, true); // 停在托盘上方
            armControl_xyz(platePos[taskcode[i] - 1].x, platePos[taskcode[i] - 1].y, platePos[taskcode[i] - 1].z, 400, 200, 200, true);                   // 下降到托盘
            arm_setClaw(0);                                                                                                                               // 闭合夹爪
            armControl_xyz(platePos[taskcode[i] - 1].x, platePos[taskcode[i] - 1].y, platePos[taskcode[i] - 1].z + overPlateHeight, 400, 200, 200, true); // 上升到托盘上方
            for (int j = 0; j < 5; j++)
            {
                armControl_xyz(keyPos[taskcode[i] - 1][j].x, keyPos[taskcode[i] - 1][j].y, keyPos[taskcode[i] - 1][j].z, 400, 200, 200, true); // 从托盘到色环的关键点
            };
            armControl_xyz(circlePos[taskcode[i] - 1].x, circlePos[taskcode[i] - 1].y, circlePos[taskcode[i] - 1].z + overCircleHeight, 400, 200, 200, true); // 到色环上方
            /*获取色环偏移量并叠加偏移量*/
            int x, y;

            // 直接使用vision模块内部的重试逻辑
            if (!visionGetCircle(&x, &y))
            {
                // 如果识别失败，使用基础位置（不进行偏移）
                DEBUG_LOG("第%d种颜色色环识别失败，使用基础位置", taskcode[i]);
                errorHandle(ERROR_CIRCLE_RECOGNITION_FAILED);
            }
            else
            {
                DEBUG_LOG("第%d种颜色色环识别成功，偏移量：x=%d, y=%d", taskcode[i], x, y);

                // 如果识别成功，叠加偏移量，先进行坐标系转换
                float current_angle = servo0.queryAngle();
                float global_x, global_y;

                // 转换摄像头局部坐标到全局坐标
                localToGlobalCoordinates(x, y, current_angle, &global_x, &global_y);

                // 使用转换后的坐标更新色环位置
                circlePos[taskcode[i] - 1].x += global_x * scale;
                circlePos[taskcode[i] - 1].y += global_y * scale;
                DEBUG_LOG("色环位置已更新，考虑云台角度%f度后的偏移量: x=%f, y=%f", current_angle, global_x * scale, global_y * scale);
            }

            armControl_xyz(circlePos[taskcode[i] - 1].x, circlePos[taskcode[i] - 1].y, circlePos[taskcode[i] - 1].z, 400, 200, 200, true); // 放到色环上
            arm_setClaw(1);
            delay(500);
            armControl_xyz(circlePos[taskcode[i] - 1].x, circlePos[taskcode[i] - 1].y, circlePos[taskcode[i] - 1].z + overCircleHeight, 400, 200, 200, true); // 上升到色环上方
            armControl_xyz(0, 93, 130, 400, 200, 200, true);
        }
        armResumeMainSequence();
        // 释放参数内存并删除任务
        free(params);
        vTaskDelete(NULL);
    };

    // 创建FreeRTOS任务
    BaseType_t result = xTaskCreate(
        taskFunction,     // lambda表达式作为任务函数
        "ArmPutToGround", // 任务名称
        4096,             // 堆栈大小
        params,           // 任务参数
        1,                // 任务优先级
        NULL              // 任务句柄
    );

    if (result != pdPASS)
    {
        DEBUG_LOG("创建arm_putToGround任务失败");
        free(params);
    }
    armPauseMainSequence();
}

void arm_catchFromGround(int taskcode[3]) // 将物料从地面抓取到托盘
{
    // 分配任务参数内存
    ArmTaskParams *params = (ArmTaskParams *)malloc(sizeof(ArmTaskParams));
    if (params == NULL)
    {
        DEBUG_LOG("内存分配失败");
        return;
    }

    // 复制参数
    for (int i = 0; i < 3; i++)
    {
        params->taskcode[i] = taskcode[i];
    }

    // 使用lambda表达式创建任务
    auto taskFunction = [](void *parameter) -> void
    {
        ArmTaskParams *params = (ArmTaskParams *)parameter;
        int *taskcode = params->taskcode;

        /*从地面抓取，获取放置时的偏差，叠加偏移量进行抓取*/
        for (int i = 0; i < 3; i++) // 循环3次，分别抓取3种颜色物料
        {
            bool gripSuccess = false;
            int retryCount = 0;
            const int maxRetries = 3; // 最大重试次数

            while (!gripSuccess && retryCount < maxRetries)
            {
                retryCount++;
                arm_setClaw(1);
                armControl_xyz(circlePos[taskcode[i] - 1].x, circlePos[taskcode[i] - 1].y, circlePos[taskcode[i] - 1].z + overCircleHeight, 400, 200, 200, true); // 上升到色环上方
                armControl_xyz(circlePos[taskcode[i] - 1].x, circlePos[taskcode[i] - 1].y, circlePos[taskcode[i] - 1].z, 400, 200, 200, true);                    // 下降到色环
                arm_setClaw(0);                                                                                                                                   // 闭合夹爪

                // 等待夹爪状态稳定
                vTaskDelay(pdMS_TO_TICKS(500));
            }

            // 无论是否成功都继续后续流程
            armControl_xyz(circlePos[taskcode[i] - 1].x, circlePos[taskcode[i] - 1].y, circlePos[taskcode[i] - 1].z + overCircleHeight, 400, 200, 200, true); // 上升到色环上方
            for (int j = 0; j < 5; j++)                                                                                                                       // 循环5次，分别从色环到托盘的关键点
            {
                armControl_xyz(keyPos[taskcode[i] + 3 - 1][j].x, keyPos[taskcode[i] + 3 - 1][j].y, keyPos[taskcode[i] + 3 - 1][j].z, 400, 200, 200, true); // 从色环到托盘的关键点
            };
            // armControl_xyz(platePos[taskcode[i]-1].x, platePos[taskcode[i]-1].y, platePos[taskcode[i]-1].z + overPlateHeight, 400, 200, 200, true); // 到托盘上方
            armControl_xyz(platePos[taskcode[i] - 1].x, platePos[taskcode[i] - 1].y, platePos[taskcode[i] - 1].z, 400, 200, 200, true); // 放到托盘上
            arm_setClaw(1);
            delay(500);
            armControl_xyz(platePos[taskcode[i] - 1].x, platePos[taskcode[i] - 1].y, platePos[taskcode[i] - 1].z + overPlateHeight, 400, 200, 200, true); // 上升到托盘上方
            armControl_xyz(0, 93, 130, 400, 200, 200, true);
        }
        armResumeMainSequence();
        // 释放参数内存并删除任务
        free(params);
        vTaskDelete(NULL);
    };

    // 创建FreeRTOS任务
    BaseType_t result = xTaskCreate(
        taskFunction,         // lambda表达式作为任务函数
        "ArmCatchFromGround", // 任务名称
        4096,                 // 堆栈大小
        params,               // 任务参数
        1,                    // 任务优先级
        NULL                  // 任务句柄
    );

    if (result != pdPASS)
    {
        DEBUG_LOG("创建arm_catchFromGround任务失败");
        free(params);
    }
    armPauseMainSequence();
}

void arm_putToMaterial(int taskcode[3]) // 将第二次的物料重合到第一次的物料的上方
{
    // 分配任务参数内存
    ArmTaskParams *params = (ArmTaskParams *)malloc(sizeof(ArmTaskParams));
    if (params == NULL)
    {
        DEBUG_LOG("内存分配失败");
        return;
    }

    // 复制参数
    for (int i = 0; i < 3; i++)
    {
        params->taskcode[i] = taskcode[i];
    }

    // 使用lambda表达式创建任务
    auto taskFunction = [](void *parameter) -> void
    {
        ArmTaskParams *params = (ArmTaskParams *)parameter;
        int *taskcode = params->taskcode;

        /*码垛，同样需要通过视觉识别物料位置，然后在预先设置的位置基础上叠加偏移量进行放置*/
        for (int i = 0; i < 3; i++)
        {
            arm_setClaw(1);
            armControl_xyz(platePos[taskcode[i] - 1].x, platePos[taskcode[i] - 1].y, platePos[taskcode[i] - 1].z + overPlateHeight, 400, 200, 200); // 停在托盘上方
            waitArm();
            armControl_xyz(platePos[taskcode[i] - 1].x, platePos[taskcode[i] - 1].y, platePos[taskcode[i] - 1].z, 400, 200, 200); // 下降到托盘
            waitArm();
            arm_setClaw(0); // 闭合夹爪
            waitArm();
            armControl_xyz(platePos[taskcode[i] - 1].x, platePos[taskcode[i] - 1].y, platePos[taskcode[i] - 1].z + overPlateHeight, 400, 200, 200); // 上升到托盘上方
            waitArm();
            for (int j = 0; j < 5; j++)
            {
                armControl_xyz(keyPos[taskcode[i] + 6 - 1][j].x, keyPos[taskcode[i] + 6 - 1][j].y, keyPos[taskcode[i] + 6 - 1][j].z, 400, 200, 200); // 从托盘到色环的关键点
            };
            waitArm();
            // armControl_xyz(circlePos[taskcode[i]-1].x, circlePos[taskcode[i]-1].y, circlePos[taskcode[i]-1].z + MATERIAL_HEIGHT + overCircleHeight, 400, 200, 200); // 到色环上方
            // waitArm();

            delay(500);
            /*获取色环偏移量并叠加偏移量*/
            int x, y;

            // 直接使用vision模块内部的重试逻辑
            if (!visionGetCircle(&x, &y))
            {
                DEBUG_LOG("第%d种颜色色环识别失败(码垛)，使用基础位置", taskcode[i]);
                errorHandle(ERROR_CIRCLE_RECOGNITION_FAILED);
            }
            else
            {
                DEBUG_LOG("第%d种颜色色环识别成功(码垛)，偏移量：x=%d, y=%d", taskcode[i], x, y);

                // 如果识别成功，叠加偏移量，先进行坐标系转换
                float current_angle = servo0.queryAngle();
                float global_x, global_y;

                // 转换摄像头局部坐标到全局坐标
                localToGlobalCoordinates(x, y, current_angle, &global_x, &global_y);

                // 使用转换后的坐标更新色环位置
                circlePos[taskcode[i]].x += global_x * scale;
                circlePos[taskcode[i]].y += global_y * scale;
                DEBUG_LOG("色环位置已更新(码垛)，考虑云台角度%f度后的偏移量: x=%f, y=%f", current_angle, global_x * scale, global_y * scale);
            }

            armControl_xyz(circlePos[taskcode[i] - 1].x, circlePos[taskcode[i] - 1].y, circlePos[taskcode[i] - 1].z + MATERIAL_HEIGHT, 400, 200, 200); // 放到色环的物料上
            arm_setClaw(1);
            delay(500);
            armControl_xyz(circlePos[taskcode[i] - 1].x, circlePos[taskcode[i] - 1].y, circlePos[taskcode[i] - 1].z + MATERIAL_HEIGHT + overCircleHeight, 400, 200, 200); // 上升到色环上方
            armControl_xyz(0, 93, 140, 400, 200, 200);
        }
        armResumeMainSequence();
        // 释放参数内存并删除任务
        free(params);
        vTaskDelete(NULL);
    };

    // 创建FreeRTOS任务
    BaseType_t result = xTaskCreate(
        taskFunction,       // lambda表达式作为任务函数
        "ArmPutToMaterial", // 任务名称
        4096,               // 堆栈大小
        params,             // 任务参数
        1,                  // 任务优先级
        NULL                // 任务句柄
    );

    if (result != pdPASS)
    {
        DEBUG_LOG("创建arm_putToMaterial任务失败");
        free(params);
    }
    armPauseMainSequence();
}

/**
 * @brief 将摄像头局部坐标系的坐标转换到全局坐标系
 * @param local_x 摄像头局部坐标系中的x坐标
 * @param local_y 摄像头局部坐标系中的y坐标
 * @param servo0_angle 云台角度（度）
 * @param global_x 转换后全局坐标系中的x坐标（输出参数）
 * @param global_y 转换后全局坐标系中的y坐标（输出参数）
 */
void localToGlobalCoordinates(float local_x, float local_y, float servo0_angle, float *global_x, float *global_y)
{
    // 将角度转换为弧度
    float angle_rad = servo0_angle * DEG_TO_RAD;

    // 坐标系转换
    *global_x = local_x * cos(angle_rad) - local_y * sin(angle_rad);
    *global_y = local_x * sin(angle_rad) + local_y * cos(angle_rad);

    DEBUG_LOG("坐标转换: 局部(%f,%f), 角度%f度, 全局(%f,%f)",
              local_x, local_y, servo0_angle, *global_x, *global_y);
}

// 机械臂控制MainSequence的推进与暂停相关函数实现
/**
 * @brief 暂停主流程
 * 通过获取并持有暂停信号量，阻止主流程继续执行
 */
void armPauseMainSequence(void)
{
    // 尝试获取信号量，如果成功则说明当前未暂停
    if (xSemaphoreTake(xSemaphoreMainsequencePause, 50) == pdTRUE)
    {
        DEBUG_LOG("机械臂控制：主流程已暂停");
        // 获取后不释放，保持暂停状态
    }
    else
    {
        DEBUG_LOG("机械臂控制：主流程已经处于暂停状态");
    }
}

/**
 * @brief 恢复/推进主流程
 * 通过释放暂停信号量，允许主流程继续执行
 */
void armResumeMainSequence(void)
{
    // 先尝试获取信号量，如果失败则说明当前已暂停
    if (xSemaphoreTake(xSemaphoreMainsequencePause, 50) == pdFAIL)
    {
        // 释放信号量，恢复主流程执行
        xSemaphoreGive(xSemaphoreMainsequencePause);
        DEBUG_LOG("机械臂控制：主流程已恢复运行");
    }
    else
    {
        // 如果成功获取，说明当前未暂停，需要释放回去
        xSemaphoreGive(xSemaphoreMainsequencePause);
        DEBUG_LOG("机械臂控制：主流程未处于暂停状态");
    }
}
