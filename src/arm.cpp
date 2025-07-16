#include "arm.h"
#include "config.h"
#include "vision.h"
#include "taskManager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include "errorHandler.h"
#if DEBUG_ENABLE
#include "displayInterface.h"  // 用于请求UI更新
#endif
FSUS_Protocol protocol(&SERVO_SERIAL, SERIAL_BAUDRATE);
FSUS_Servo servo0(0, &protocol); // 云台舵机
FSUS_Servo servo1(1, &protocol); // 一级关节舵机
FSUS_Servo servo2(2, &protocol); // 二级关节舵机
FSUS_Servo servo3(3, &protocol); // 三级关节舵机
FSUS_Servo servo4(4, &protocol); // 夹爪舵机

// 夹爪位置监测相关全局变量
bool gripperHolding = false;         // 夹持状态标志，false表示未夹持，true表示已夹持
float currentGripperAngle = 0.0f;    // 当前夹爪角度，单位度
TaskHandle_t gripperMonitorTaskHandle = NULL; // 夹爪监测任务句柄
SemaphoreHandle_t gripperMutex = NULL; // 夹爪状态互斥锁

const float scale = 0.1f;

struct armPos
{
    int x;
    int y;
    int z;
};
const armPos fold = {35,0, 99};        // 折叠状态
const armPos ttDetect = {240, 0, 240};    // 转盘检测位置
const armPos rPlate = {65, -62, 90};      // 红色物料托盘位置
const armPos bPlate = {-67, 63, 88};      // 蓝色物料托盘位置
const armPos gPlate = {61, 62, 91};      // 绿色物料托盘位置
const int plateAngle[3] = {136,46,-46}; // 托盘角度
const armPos roverthecircle = {0,0,0}; //红色物料空中位置
const armPos boverthecircle = {0,0,0}; //蓝色物料空中位置
const armPos goverthecircle = {0,0,0}; //绿色物料空中位置
//const armPos gdDetect = {0, 0, 0};    // 地面检测位置
const armPos rCircleBase = {145, 220, 0}; // 红色色环基础位置（未视觉纠偏的位置）
const armPos bCircleBase = {-155,200, 0}; // 蓝色色环基础位置（未视觉纠偏的位置）
const armPos gCircleBase = {-6, 210, 0}; // 绿色色环基础位置（未视觉纠偏的位置）

armPos platePos[3] = {rPlate,gPlate,bPlate};
armPos circlePos[3] = {rCircleBase,gCircleBase,bCircleBase};

armPos keyPos[12][5]={
    {//红色 从托盘到色环关键点
        {67,-64,120},{75,0,161},{82,60,200},{107,144,158},{117,170,110}
    },
    {//绿色 从托盘到色环关键点
        {64,66,136},{50,100,125},{20,145,106},{0,166,80},{0,187,42}
    },
    {//蓝色 从托盘到色环关键点
        {-71,66,120},{-100,100,120},{-125,150,95  },{-135,170,65},{-145,178,60}
    },
    {//红色 从色环到托盘关键点
        {117,170,110},{107,144,158},{82,60,200},{75,0,161},{67,-64,120}
    },
    {//绿色 从色环到托盘关键点
        {0,187,42},{0,166,80},{20,145,106},{50,100,125},{64,66,136}
    },
    {//蓝色 从色环到托盘关键点
        {-145,178,60},{-135,170,65},{-125,150,95},{-100,100,120},{-71,66,120}
    },
    {//红色 从托盘到色环上的物料的关键点
        {67,-64,120},{75,0,161},{82,60,200},{107,144,158},{117,170,110}
    },
    {//绿色 从托盘到色环上的物料的关键点
        {64,66,136},{50,100,125},{20,145,115},{0,166,100},{0,187,90}
    },
    {//蓝色 从托盘到色环上的物料的关键点
        {-71,66,120},{-100,100,120},{-125,150,95 },{-135,170,85},{-145,178,80}
    }
};


const int overPlateHeight = 50; // 机器爪抓取物料之前高于物料的高度
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
    //servo4.SetOriginPoint();//夹爪舵机一般不需要归零
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
        hypotenuse < fabs(ARM_FIRST_LENGTH - ARM_SECOND_LENGTH)) {
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

    y1 = w * cos(theta0 * DEG_TO_RAD); // 将w轴增量转换到机械臂中心坐标系下的y轴增量
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
    servo0.setAngle(theta0, interval, acc, dec);                   // 云台舵机
    delay(3);
    servo1.setAngle(first_arm_degree - 1.0f, interval, acc, dec);  // 一级关节舵机
    delay(3);
    servo2.setAngle(second_arm_degree - 0.5f, interval, acc, dec); // 二级关节舵机
    delay(3);
    servo3.setAngle(third_arm_degree, interval, acc, dec);         // 三级关节舵机
}
/**
 *@brief 机械臂xyz平面位置控制
 *@param x: x坐标,单位mm
 *@param y: y坐标,单位mm
 *@param z: z坐标,单位mm
 *@return void
 */
void armControl_xyz(float x, float y, float z, uint16_t interval, uint16_t acc, uint16_t dec)
{
    float xyz_to_angle[4];

    if(armCalculate_inverse(x, y, z, xyz_to_angle)) // 逆解算
    {
    //DEBUG_LOG("机械臂xyz坐标: x=%f, y=%f, z=%f", x, y, z);
    armSet_position(xyz_to_angle[0], xyz_to_angle[1], xyz_to_angle[2], xyz_to_angle[3], interval, acc, dec);
    }
}

void arm_ScanQRcode()
{
    /*扫描二维码，与控制xyz不同，机械臂前端需抬起使摄像头朝向二维码，无需处理视觉部分*/
    armSet_position(90.0,-50.0,50.0,-64.0,500,100,100);
    arm_setClaw(1);
}
void arm_setClaw(bool open)
{
    if(open)
    {
        servo4.setAngle(ARM_GRIPPER_OPEN_ANGLE, 100);
        
        // 张开夹爪时，使用互斥锁保护状态修改
        if (gripperMutex != NULL && xSemaphoreTake(gripperMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            gripperHolding = false;
            xSemaphoreGive(gripperMutex);
            DEBUG_LOG("夹爪张开，重置夹持状态");
        }
        else
        {
            DEBUG_LOG("获取夹爪状态互斥锁超时");
            gripperHolding = false; // 降级处理
        }
        
        #if DEBUG_ENABLE
        requestGripperColorUpdate();  // 请求UI更新
        #endif
    }
    else
    {
        servo4.setAngle(ARM_GRIPPER_CLOSE_ANGLE, 100);
        // 注意：闭合夹爪时不立即设置夹持状态，让监测任务自动检测
    }
}
void waitArm(void){
    servo0.wait();
    servo1.wait();
    servo2.wait();
    servo3.wait();
    servo4.wait();
}

// 任务参数结构体
typedef struct {
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
        float y_offset ;
        float x_offset ;
        float servo0_angle;
        waitArm();
        for (int i = 0; i < 3; i++)//循环3次，分别抓取3种颜色物料
        {
            bool gripSuccess = false;  // 夹取成功标志
            int retryCount = 0;        // 重试计数器
            const int maxRetries = 3;  // 最大重试次数
            
            while (!gripSuccess && retryCount < maxRetries)
            {
                retryCount++;
                DEBUG_LOG("正在抓取第%d种颜色物料，第%d次尝试", taskcode[i], retryCount);
                
                armControl_xyz(ttDetect.x, ttDetect.y, ttDetect.z, 1000, 100, 100); // 停在转盘中心，xyz的单位为mm
                arm_setClaw(1); // 张开夹爪以便检测物料
                waitArm();
                startTime = millis();
                // 以上一次的xy为基础叠加摄像头数据进行跟踪，若进入死区持续一定时间或超时则退出循环执行抓取
                int follow_x = ttDetect.x;
                int follow_y = ttDetect.y;
                const int deadzone = 20; // 死区阈值，单位与x/y一致
                const int deadzone_time = 1000; // 死区持续时间ms
                int deadzone_start = 0;
                bool in_deadzone = false;
                while (1)
                {
                    if (millis() - startTime > VISION_GET_MATERIAL_TIMEOUT)
                    {
                        errorHandle(ERROR_MATERIAL_RECOGNITION_FAILED);
                        break;
                    }
                    visionGetMaterial(taskcode[i], &x, &y); // 视觉识别，x,y为物料在摄像头中的坐标
                    follow_x += x * scale;
                    follow_y += y * scale;
                    armControl_xyz(follow_x, follow_y, ttDetect.z - traceHeightOffset, 1000, 100, 100);
                    waitArm();

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
                armControl_xyz(ttDetect.x + x * scale + x_offset, ttDetect.y + y * scale + y_offset, turntableHeight, 1000, 100, 100);
                waitArm();
                arm_setClaw(0); // 闭合夹爪
                waitArm();
                
                // 等待夹爪状态稳定并验证夹取是否成功
                vTaskDelay(pdMS_TO_TICKS(100)); // 等待500ms让监测任务检测状态
                
                if (isGripperHolding())
                {
                    gripSuccess = true;
                    DEBUG_LOG("第%d种颜色物料夹取成功，第%d次尝试", taskcode[i], retryCount);
                }
                else
                {
                    DEBUG_LOG("第%d种颜色物料夹取失败，第%d次尝试", taskcode[i], retryCount);
                    
                    if (retryCount < maxRetries)
                    {
                        // 如果还有重试机会，先张开夹爪，稍作等待后重试
                        arm_setClaw(1); // 张开夹爪
                        waitArm();
                        vTaskDelay(pdMS_TO_TICKS(200)); // 等待200ms
                        DEBUG_LOG("准备重试夹取第%d种颜色物料", taskcode[i]);
                        continue; // 重新开始while循环
                    }
                    else
                    {
                        DEBUG_LOG("第%d种颜色物料夹取失败，已达到最大重试次数", taskcode[i]);
                        errorHandle(ERROR_MATERIAL_RECOGNITION_FAILED);
                        break; // 跳出while循环，继续下一个物料
                    }
                }
            }
            
            // 如果夹取成功，继续后续的移动和放置操作
            if (gripSuccess)
            {
            arm_x = ttDetect.x + x * scale + x_offset;
            arm_y = ttDetect.y + y * scale + y_offset;
            if (arm_x < 280)
            {
                armControl_xyz(arm_x , arm_y , turntableHeight + 100, 1000, 100, 100);
                waitArm();
                armControl_xyz( 85 , 0 , 180 , 1000, 100, 100);
                waitArm();
                servo0.setAngle(plateAngle[taskcode[i] - 1], 500, 250, 250);
                waitArm();
                armControl_xyz(platePos[taskcode[i] - 1].x, platePos[taskcode[i] - 1].y, platePos[taskcode[i] - 1].z, 1000, 100, 100);
                waitArm();
                arm_setClaw(1);
                waitArm();
                armControl_xyz(platePos[taskcode[i] - 1].x, platePos[taskcode[i] - 1].y, platePos[taskcode[i] - 1].z + 100, 1000, 100, 100);
                waitArm();
                servo0.setAngle(90.0f, 500, 250, 250);
                waitArm();
            }
            else
            {
                armControl_xyz(ttDetect.x, ttDetect.y, turntableHeight, 1000, 100, 100);
                waitArm();
                armControl_xyz(arm_x , arm_y , turntableHeight + 100, 1000, 100, 100);
                waitArm();
                armControl_xyz( 85 , 0 , 180 , 1000, 100, 100);
                waitArm();
                servo0.setAngle(plateAngle[taskcode[i] - 1], 500, 250, 250);
                waitArm();
                armControl_xyz(platePos[taskcode[i] - 1].x, platePos[taskcode[i] - 1].y, platePos[taskcode[i] - 1].z, 1000, 100, 100);
                waitArm();
                arm_setClaw(1);
                waitArm();
                armControl_xyz(platePos[taskcode[i] - 1].x, platePos[taskcode[i] - 1].y, platePos[taskcode[i] - 1].z + 100, 1000, 100, 100);
                waitArm();
                servo0.setAngle(90.0f, 500, 250, 250);
                waitArm();
            }
            } // 关闭 if (gripSuccess) 块
        } // 关闭 for 循环

        // 释放参数内存并删除任务
        free(params);
        vTaskDelete(NULL);
    };

    // 创建FreeRTOS任务
    BaseType_t result = xTaskCreate(
        taskFunction,              // lambda表达式作为任务函数
        "ArmCatchFromTurntable",   // 任务名称
        4096,                      // 堆栈大小
        params,                    // 任务参数
        1,                         // 任务优先级
        NULL                       // 任务句柄
    );
    
    if (result != pdPASS) {
        DEBUG_LOG("创建arm_catchFromTurntable任务失败");
        free(params);
    }
}

void arm_putToGround(int taskcode[3])//将第一次的物料放置到地面的色环
{
    // 分配任务参数内存
    ArmTaskParams* params = (ArmTaskParams*)malloc(sizeof(ArmTaskParams));
    if (params == NULL) {
        DEBUG_LOG("内存分配失败");
        return;
    }
    
    // 复制参数
    for (int i = 0; i < 3; i++) {
        params->taskcode[i] = taskcode[i];
    }
    
    // 使用lambda表达式创建任务
    auto taskFunction = [](void* parameter) -> void {
        ArmTaskParams* params = (ArmTaskParams*)parameter;
        int* taskcode = params->taskcode;
        
        /*放置到地面，需要通过视觉识别放置位置，然后在预先设置的位置基础上叠加偏移量进行放置*/
        for (int i = 0; i < 3; i++)
        {
            arm_setClaw(1);
            armControl_xyz(platePos[taskcode[i]-1].x, platePos[taskcode[i]-1].y, platePos[taskcode[i]-1].z + overPlateHeight, 400, 200, 200); // 停在托盘上方
            waitArm();
            armControl_xyz(platePos[taskcode[i]-1].x, platePos[taskcode[i]-1].y, platePos[taskcode[i]-1].z,400, 200, 200); // 下降到托盘
            waitArm();
            arm_setClaw(0); // 闭合夹爪
            waitArm();
            armControl_xyz(platePos[taskcode[i]-1].x, platePos[taskcode[i]-1].y, platePos[taskcode[i]-1].z + overPlateHeight, 400, 200, 200); // 上升到托盘上方
            waitArm();
            for (int j = 0; j < 5; j++)
            {
                armControl_xyz(keyPos[taskcode[i]-1][j].x, keyPos[taskcode[i]-1][j].y, keyPos[taskcode[i]-1][j].z, 400, 200, 200); // 从托盘到色环的关键点
            };
            waitArm();
            armControl_xyz(circlePos[taskcode[i]-1].x, circlePos[taskcode[i]-1].y, circlePos[taskcode[i]-1].z + overCircleHeight, 400, 200, 200); // 到色环上方
            waitArm();

           delay(500);
            /*获取色环偏移量并叠加偏移量*/
            int x, y;
            visionGetCircle(&x, &y);
            circlePos[taskcode[i]-1].x += x * scale;
            circlePos[taskcode[i]-1].y += y * scale;
    
            armControl_xyz(circlePos[taskcode[i]-1].x, circlePos[taskcode[i]-1].y, circlePos[taskcode[i]-1].z, 400, 200, 200); // 放到色环上
            waitArm();
            arm_setClaw(1);
            waitArm();
            armControl_xyz(circlePos[taskcode[i]-1].x, circlePos[taskcode[i]-1].y, circlePos[taskcode[i]-1].z + overCircleHeight, 400, 200, 200); // 上升到色环上方
            waitArm();
            armControl_xyz(0, 93, 130, 400, 200, 200);
            waitArm();
        }
        
        // 释放参数内存并删除任务
        free(params);
        vTaskDelete(NULL);
    };
    
    // 创建FreeRTOS任务
    BaseType_t result = xTaskCreate(
        taskFunction,              // lambda表达式作为任务函数
        "ArmPutToGround",          // 任务名称
        4096,                      // 堆栈大小
        params,                    // 任务参数
        1,                         // 任务优先级
        NULL                       // 任务句柄
    );
    
    if (result != pdPASS) {
        DEBUG_LOG("创建arm_putToGround任务失败");
        free(params);
    }
}

void arm_catchFromGround(int taskcode[3])//将物料从地面抓取到托盘
{
    // 分配任务参数内存
    ArmTaskParams* params = (ArmTaskParams*)malloc(sizeof(ArmTaskParams));
    if (params == NULL) {
        DEBUG_LOG("内存分配失败");
        return;
    }
    
    // 复制参数
    for (int i = 0; i < 3; i++) {
        params->taskcode[i] = taskcode[i];
    }
    
    // 使用lambda表达式创建任务
    auto taskFunction = [](void* parameter) -> void {
        ArmTaskParams* params = (ArmTaskParams*)parameter;
        int* taskcode = params->taskcode;
        
        /*从地面抓取，获取放置时的偏差，叠加偏移量进行抓取*/
        for (int i = 0; i < 3; i++)// 循环3次，分别抓取3种颜色物料
        {
            arm_setClaw(1);
            armControl_xyz(circlePos[taskcode[i]-1].x, circlePos[taskcode[i]-1].y, circlePos[taskcode[i]-1].z + overCircleHeight, 400, 200, 200); // 上升到色环上方
            waitArm();
            armControl_xyz(circlePos[taskcode[i]-1].x, circlePos[taskcode[i]-1].y, circlePos[taskcode[i]-1].z, 400, 200, 200); // 下降到色环
            waitArm();
            arm_setClaw(0); // 闭合夹爪
            waitArm();
            armControl_xyz(circlePos[taskcode[i]-1].x, circlePos[taskcode[i]-1].y, circlePos[taskcode[i]-1].z + overCircleHeight, 400, 200, 200); // 上升到色环上方
            waitArm();
            for (int j = 0; j < 5; j++)// 循环5次，分别从色环到托盘的关键点
            {                                                                                                                             
                armControl_xyz(keyPos[taskcode[i] + 3-1][j].x, keyPos[taskcode[i] + 3-1][j].y, keyPos[taskcode[i] + 3-1][j].z, 400, 200, 200); // 从色环到托盘的关键点
            };
            waitArm();
            //armControl_xyz(platePos[taskcode[i]-1].x, platePos[taskcode[i]-1].y, platePos[taskcode[i]-1].z + overPlateHeight, 400, 200, 200); // 到托盘上方
            //waitArm();
            armControl_xyz(platePos[taskcode[i]-1].x, platePos[taskcode[i]-1].y, platePos[taskcode[i]-1].z, 400, 200, 200); // 放到托盘上
            waitArm();
            arm_setClaw(1);
            waitArm();
            armControl_xyz(platePos[taskcode[i]-1].x, platePos[taskcode[i]-1].y, platePos[taskcode[i]-1].z + overPlateHeight, 400, 200, 200); // 上升到托盘上方
            waitArm();
            armControl_xyz(0, 93, 130, 400, 200, 200);
            waitArm();
        }
        
        // 释放参数内存并删除任务
        free(params);
        vTaskDelete(NULL);
    };
    
    // 创建FreeRTOS任务
    BaseType_t result = xTaskCreate(
        taskFunction,              // lambda表达式作为任务函数
        "ArmCatchFromGround",      // 任务名称
        4096,                      // 堆栈大小
        params,                    // 任务参数
        1,                         // 任务优先级
        NULL                       // 任务句柄
    );
    
    if (result != pdPASS) {
        DEBUG_LOG("创建arm_catchFromGround任务失败");
        free(params);
    }
}

void arm_putToMaterial(int taskcode[3])//将第二次的物料重合到第一次的物料的上方
{
    // 分配任务参数内存
    ArmTaskParams* params = (ArmTaskParams*)malloc(sizeof(ArmTaskParams));
    if (params == NULL) {
        DEBUG_LOG("内存分配失败");
        return;
    }
    
    // 复制参数
    for (int i = 0; i < 3; i++) {
        params->taskcode[i] = taskcode[i];
    }
    
    // 使用lambda表达式创建任务
    auto taskFunction = [](void* parameter) -> void {
        ArmTaskParams* params = (ArmTaskParams*)parameter;
        int* taskcode = params->taskcode;
        
        /*码垛，同样需要通过视觉识别物料位置，然后在预先设置的位置基础上叠加偏移量进行放置*/
        for (int i = 0; i < 3; i++)
        {
            arm_setClaw(1);
            armControl_xyz(platePos[taskcode[i]-1].x, platePos[taskcode[i]-1].y, platePos[taskcode[i]-1].z + overPlateHeight, 400, 200, 200); // 停在托盘上方
            waitArm();
            armControl_xyz(platePos[taskcode[i]-1].x, platePos[taskcode[i]-1].y, platePos[taskcode[i]-1].z, 400, 200, 200); // 下降到托盘
            waitArm();
            arm_setClaw(0); // 闭合夹爪
            waitArm();
            armControl_xyz(platePos[taskcode[i]-1].x, platePos[taskcode[i]-1].y, platePos[taskcode[i]-1].z + overPlateHeight, 400, 200, 200); // 上升到托盘上方
            waitArm();
            for (int j = 0; j < 5; j++)
            {
                armControl_xyz(keyPos[taskcode[i]+6-1][j].x, keyPos[taskcode[i]+6-1][j].y, keyPos[taskcode[i]+6-1][j].z, 400, 200, 200); // 从托盘到色环的关键点
            };
            waitArm();
            //armControl_xyz(circlePos[taskcode[i]-1].x, circlePos[taskcode[i]-1].y, circlePos[taskcode[i]-1].z + MATERIAL_HEIGHT + overCircleHeight, 400, 200, 200); // 到色环上方
            //waitArm();

            delay(500);
            /*获取色环偏移量并叠加偏移量*/
            int x, y;
            visionGetCircle(&x, &y);
            circlePos[taskcode[i]].x += x*scale;
            circlePos[taskcode[i]].y += y*scale;

            armControl_xyz(circlePos[taskcode[i]-1].x, circlePos[taskcode[i]-1].y, circlePos[taskcode[i]-1].z + MATERIAL_HEIGHT, 400, 200, 200); // 放到色环的物料上
            waitArm();
            arm_setClaw(1);
            waitArm();
            armControl_xyz(circlePos[taskcode[i]-1].x, circlePos[taskcode[i]-1].y, circlePos[taskcode[i]-1].z + MATERIAL_HEIGHT +  overCircleHeight, 400, 200, 200); // 上升到色环上方
            waitArm();
            armControl_xyz(0, 93, 140, 400, 200, 200);
        }
        
        // 释放参数内存并删除任务
        free(params);
        vTaskDelete(NULL);
    };
    
    // 创建FreeRTOS任务
    BaseType_t result = xTaskCreate(
        taskFunction,              // lambda表达式作为任务函数
        "ArmPutToMaterial",        // 任务名称
        4096,                      // 堆栈大小
        params,                    // 任务参数
        1,                         // 任务优先级
        NULL                       // 任务句柄
    );
    
    if (result != pdPASS) {
        DEBUG_LOG("创建arm_putToMaterial任务失败");
        free(params);
    }
}

/******************************************************************************
 * 夹爪位置监测相关函数实现
 ******************************************************************************/

/**
 * @brief 夹爪监测任务 - 持续监测夹爪的角度位置来判断夹持状态
 * @param pvParameters 任务参数
 */
void gripperMonitorTask(void *pvParameters)
{
    int stableHoldingCount = 0;     // 连续检测到夹持状态的次数
    int stableReleaseCount = 0;     // 连续检测到释放状态的次数
    
    DEBUG_LOG("夹爪位置监测任务已启动");
    
    while (1)
    {
        // 查询夹爪舵机的当前角度
        servo4.queryAngle();
        float angle = servo4.curAngle;
        
        // 更新全局变量
        currentGripperAngle = angle;
        
        // 判断夹持状态基于角度位置
        // 角度大于-6.5度时认为夹持到物料（因为遇到阻力无法完全闭合）
        // 角度在40度附近时认为是张开状态
        bool isOpenPosition = (angle > (GRIPPER_OPEN_ANGLE_THRESHOLD - 5.0f)); // 35度以上为张开状态
        bool currentlyHolding = false;
        
        if (!isOpenPosition) {
            // 在闭合状态下，角度大于阈值表示夹持到物料
            currentlyHolding = (angle > GRIPPER_HOLD_ANGLE_THRESHOLD);
        }
        
        // 使用互斥锁保护状态读取和修改
        bool holdingStatus = false;
        if (gripperMutex != NULL && xSemaphoreTake(gripperMutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            holdingStatus = gripperHolding;
            xSemaphoreGive(gripperMutex);
        }
        else
        {
            // 获取锁失败，使用当前值（可能不是最新的）
            holdingStatus = gripperHolding;
        }
        
        if (currentlyHolding && !holdingStatus)
        {
            // 当前检测到夹持，但之前状态是未夹持
            stableHoldingCount++;
            stableReleaseCount = 0;
            
            if (stableHoldingCount >= GRIPPER_STABLE_COUNT)
            {
                // 使用互斥锁保护状态修改
                if (gripperMutex != NULL && xSemaphoreTake(gripperMutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    gripperHolding = true;
                    xSemaphoreGive(gripperMutex);
                    DEBUG_LOG("检测到夹持物料 - 角度: %.1f°", currentGripperAngle);
                    stableHoldingCount = 0;
                    #if DEBUG_ENABLE
                    requestGripperColorUpdate();  // 请求UI更新
                    #endif
                }
            }
        }
        else if (!currentlyHolding && holdingStatus)
        {
            // 当前检测到释放，但之前状态是夹持
            stableReleaseCount++;
            stableHoldingCount = 0;
            
            if (stableReleaseCount >= GRIPPER_STABLE_COUNT)
            {
                // 使用互斥锁保护状态修改
                if (gripperMutex != NULL && xSemaphoreTake(gripperMutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    gripperHolding = false;
                    xSemaphoreGive(gripperMutex);
                    DEBUG_LOG("检测到释放物料 - 角度: %.1f°", currentGripperAngle);
                    stableReleaseCount = 0;
                    #if DEBUG_ENABLE
                    requestGripperColorUpdate();  // 请求UI更新
                    #endif
                }
            }
        }
        else
        {
            // 状态稳定，重置计数器
            stableHoldingCount = 0;
            stableReleaseCount = 0;
        }
        
        // 等待指定的监测间隔
        vTaskDelay(pdMS_TO_TICKS(GRIPPER_MONITOR_INTERVAL));
    }
}

/**
 * @brief 初始化夹爪监测任务
 */
void initGripperMonitor(void)
{
    // 创建互斥锁
    gripperMutex = xSemaphoreCreateMutex();
    if (gripperMutex == NULL)
    {
        DEBUG_LOG("创建夹爪状态互斥锁失败");
        return;
    }
    
    // 初始化状态
    gripperHolding = false;
    currentGripperAngle = 0.0f;
    
    // 创建夹爪监测任务
    BaseType_t result = xTaskCreate(
        gripperMonitorTask,          // 任务函数
        "GripperMonitor",           // 任务名称
        4096,                       // 堆栈大小 (增加到4096字节以避免堆栈溢出)
        NULL,                       // 任务参数
        1,                          // 任务优先级（普通优先级即可）
        &gripperMonitorTaskHandle   // 任务句柄
    );
    
    if (result != pdPASS) {
        DEBUG_LOG("创建夹爪位置监测任务失败");
        gripperMonitorTaskHandle = NULL;
        // 如果任务创建失败，删除互斥锁
        if (gripperMutex != NULL)
        {
            vSemaphoreDelete(gripperMutex);
            gripperMutex = NULL;
        }
    } else {
        DEBUG_LOG("夹爪位置监测任务和互斥锁初始化成功");
        #if DEBUG_ENABLE
        // 初始化UI组件颜色
        vTaskDelay(pdMS_TO_TICKS(500));  // 等待UI初始化完成
        requestGripperColorUpdate();
        #endif
    }
}

/**
 * @brief 获取当前夹持状态
 * @return true表示正在夹持物料，false表示未夹持
 */
bool isGripperHolding(void)
{
    bool status = false;
    
    // 使用互斥锁保护状态读取
    if (gripperMutex != NULL && xSemaphoreTake(gripperMutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        status = gripperHolding;
        xSemaphoreGive(gripperMutex);
    }
    else
    {
        // 获取锁失败，使用当前值（可能不是最新的）
        status = gripperHolding;
        DEBUG_LOG("获取夹爪状态时互斥锁超时");
    }
    
    return status;
}

/**
 * @brief 停止夹爪监测任务
 */
void stopGripperMonitor(void)
{
    if (gripperMonitorTaskHandle != NULL)
    {
        vTaskDelete(gripperMonitorTaskHandle);
        gripperMonitorTaskHandle = NULL;
        DEBUG_LOG("夹爪位置监测任务已停止");
    }
    
    // 销毁互斥锁
    if (gripperMutex != NULL)
    {
        vSemaphoreDelete(gripperMutex);
        gripperMutex = NULL;
        DEBUG_LOG("夹爪状态互斥锁已销毁");
    }
}
