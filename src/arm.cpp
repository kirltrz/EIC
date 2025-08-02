#include "arm.h"
#include "config.h"
#include "vision.h"
#include "taskManager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include "errorHandler.h"
#include "VOFAdebug.h"
#include "sensor.h"
#include "mainSequence.h"

FSUS_Protocol protocol(&SERVO_SERIAL, SERIAL_BAUDRATE);
FSUS_Servo servo0(0, &protocol); // 云台舵机
FSUS_Servo servo1(1, &protocol); // 一级关节舵机
FSUS_Servo servo2(2, &protocol); // 二级关节舵机
FSUS_Servo servo3(3, &protocol); // 三级关节舵机
FSUS_Servo servo4(4, &protocol); // 夹爪舵机

const float scale = 0.2f;

struct armPos
{
    int x;
    int y;
    int z;
};
struct armPosRaw
{
    float servo0Angle;
    float servo1Angle;
    float servo2Angle;
    float servo3Angle;
};

const armPos fold = {35,0, 99};        // 折叠状态
const armPosRaw ttDetect = {90.0f, 2.7f, -41.9f, 25.8f}; // 转盘检测位置
const armPos rPlate = {67, -65, 95};      // 红色物料托盘位置
const armPos bPlate = {-67, 65, 95};      // 蓝色物料托盘位置
const armPos gPlate = {67, 67, 95};      // 绿色物料托盘位置
const int plateAngle[3] = {136,46,-46}; // 托盘角度
const armPos gdDetect = {0, 180, 60};    // 地面检测位置
const armPos rCircleBase = {145, 220, 0}; // 红色色环基础位置（未视觉纠偏的位置）
const armPos bCircleBase = {-155,200, 0}; // 蓝色色环基础位置（未视觉纠偏的位置）
const armPos gCircleBase = {-6, 210, 0}; // 绿色色环基础位置（未视觉纠偏的位置）

armPos turntablePos[3] = {
    {161, -7, TURNTABLE_HEIGHT},
    {318, 80, TURNTABLE_HEIGHT},
    {317, -97, TURNTABLE_HEIGHT}};
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
const int overCircleHeight = 30; // 机器爪放置物料后高于色环的高度

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
    servo1.setAngle(first_arm_degree - 1.0f/*补偿重力*/, interval, acc, dec);  // 一级关节舵机
    delay(3);
    servo2.setAngle(second_arm_degree - 0.5f/*补偿重力*/, interval, acc, dec); // 二级关节舵机
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
void armControl_xyz(float x, float y, float z, uint16_t interval, uint16_t acc, uint16_t dec ,bool needWait)
{
    float xyz_to_angle[4];

    if(armCalculate_inverse(x, y, z, xyz_to_angle)) // 逆解算
    {
    //DEBUG_LOG("机械臂xyz坐标: x=%f, y=%f, z=%f", x, y, z);
    armSet_position(xyz_to_angle[0], xyz_to_angle[1], xyz_to_angle[2], xyz_to_angle[3], interval, acc, dec);
    }
    if(needWait){
        if(!waitArm()){
        // 检查舵机角度误差，如果大于10度则重新设置角度
        FSUS_Servo* servos[5] = {&servo0, &servo1, &servo2, &servo3, &servo4};
        for (int i = 0; i < 5; i++) {
            if (servos[i]->isMTurn) {
                servos[i]->queryRawAngleMTurn();
            } else {
                servos[i]->queryRawAngle();
            }
            
            float error = abs(servos[i]->curRawAngle - servos[i]->targetRawAngle);
            if (error > 10.0f) {
                // 误差大于10度，重新设置目标角度
                servos[i]->setAngle(servos[i]->targetRawAngle, interval, acc, dec);
                delay(3);
            }
        }
        waitArm();//可能需要更多的重试、机械臂重新失能，看具体效果
        }
    }
}

void armControl_raw(armPosRaw armPosRaw, uint16_t interval, uint16_t acc, uint16_t dec, bool needWait){

    //DEBUG_LOG("机械臂xyz坐标: x=%f, y=%f, z=%f", x, y, z);
    armSet_position(armPosRaw.servo0Angle, armPosRaw.servo1Angle, armPosRaw.servo2Angle, armPosRaw.servo3Angle,
                    interval, acc, dec);

    if(needWait){
        if(!waitArm()){
        // 检查舵机角度误差，如果大于10度则重新设置角度
        FSUS_Servo* servos[5] = {&servo0, &servo1, &servo2, &servo3, &servo4};
        for (int i = 0; i < 5; i++) {
            if (servos[i]->isMTurn) {
                servos[i]->queryRawAngleMTurn();
            } else {
                servos[i]->queryRawAngle();
            }
            
            float error = abs(servos[i]->curRawAngle - servos[i]->targetRawAngle);
            if (error > 10.0f) {
                // 误差大于10度，重新设置目标角度
                servos[i]->setAngle(servos[i]->targetRawAngle, interval, acc, dec);
                delay(3);
            }
        }
        waitArm();//可能需要更多的重试、机械臂重新失能，看具体效果
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
    if(open)
        servo4.setAngle(ARM_GRIPPER_OPEN_ANGLE, 100);
    else
        servo4.setAngle(ARM_GRIPPER_CLOSE_ANGLE, 100);
}
bool waitArm(void){
    const uint16_t timeout_ms = 3000;
    const float deadzone = 3.0f;
    const int MAX_STABLE_COUNT = 3;  // 连续稳定次数
    const int CHECK_INTERVAL = 50;   // 检查间隔ms
    
    uint32_t start_time = millis();
    int stable_count[5] = {0, 0, 0, 0, 0};  // 每个舵机的稳定计数
    FSUS_Servo* servos[5] = {&servo0, &servo1, &servo2, &servo3, &servo4};
    bool servo_stopped[5] = {false, false, false, false, false};
    
    while (millis() - start_time < timeout_ms) {
        bool all_stopped = true;
        
        // 并行检查所有舵机状态
        for (int i = 0; i < 5; i++) {
            if (!servo_stopped[i]) {
                // 查询当前角度
                if (servos[i]->isMTurn) {
                    servos[i]->queryRawAngleMTurn();
                } else {
                    servos[i]->queryRawAngle();
                }
                
                // 检查是否在死区内
                float error = abs(servos[i]->curRawAngle - servos[i]->targetRawAngle);
                if (error <= deadzone) {
                    stable_count[i]++;
                    if (stable_count[i] >= MAX_STABLE_COUNT) {
                        servo_stopped[i] = true;
                    }
                } else {
                    stable_count[i] = 0;  // 重置稳定计数
                }
                
                if (!servo_stopped[i]) {
                    all_stopped = false;
                }
            }
        }
        
        if (all_stopped) {
            DEBUG_LOG("所有舵机已到位，等待时间: %dms", millis() - start_time);
            break;
        }
        
        delay(CHECK_INTERVAL);
    }
    
    if (millis() - start_time >= timeout_ms) {
        DEBUG_LOG("等待超时，部分舵机可能未到位");
        return false;
    }
    return true;
}

void GOGOGO()
{
    // 恢复主流程任务
    if (xTaskHandleMainSequence != NULL)
    {
        vTaskResume(xTaskHandleMainSequence);
        DEBUG_LOG("主流程任务已恢复");
    }
}
// 任务参数结构体
typedef struct {
    int taskcode[3];
    bool isRoughArea; // 是否在粗加工区（true=粗加工区，false=暂存区）
    int cycleNum; // 第一轮在暂存区放在地面，第二轮码垛
} ArmTaskParams;

// 简化的物料抓取结构体
struct MaterialInfo {
    int turntablePos;  // 转盘位置索引 (0,1,2)
    int plateColor;    // 托盘颜色索引 (0,1,2 对应 R,G,B)
};

void catchFromTurntable(int sequence/*抓取序号：0=第一次，1=中间，2=最后一次*/, MaterialInfo material){
    // 根据序号确定是否为第一次或最后一次
    bool isFirst = (sequence == 0);
    bool isLast = (sequence == 2);
    
    armControl_xyz(
        turntablePos[material.turntablePos].x,
        turntablePos[material.turntablePos].y,
        turntablePos[material.turntablePos].z + (material.turntablePos == 0 ? 0 : /*重力补偿*/ 15),
        600, 300, 300);
    arm_setClaw(0);
    waitArm();
    armControl_xyz( 85 , 0 , 180 , material.turntablePos == 0 ? 300 : 600, 150, 150, false);
    delay(material.turntablePos == 0 ? 200 : 500);
    servo0.setAngle(plateAngle[material.plateColor], material.plateColor == 2/*蓝色远一些*/ ? 500 : 300, 150, 150);
    delay(material.plateColor == 2/*蓝色远一些*/ ? 450 : 250);
    armControl_xyz(platePos[material.plateColor].x, platePos[material.plateColor].y, platePos[material.plateColor].z, 300, 150, 150);
    arm_setClaw(1);
    waitArm();
    if(isLast) GOGOGO();//最后一次放完，恢复主流程任务，无需等待复位
    armControl_xyz(platePos[material.plateColor].x, platePos[material.plateColor].y, platePos[material.plateColor].z + 100, 200, 100, 100, false);
    delay(100);
    if(isLast)
        arm_groundDetect();
    else{
        servo0.setAngle(90.0f, material.plateColor == 2/*蓝色远一些*/ ? 500 : 300, 150, 150);
        delay(material.plateColor == 2/*蓝色远一些*/ ? 400 : 200);
        arm_turntableDetect();
    }
}
void arm_turntableDetect(void){
    arm_setClaw(1);
    armControl_raw(ttDetect, 600, 300, 300, false);
    delay(500);
}
// 将xy坐标转换为极坐标角度的函数
// 参数：target_x, target_y - 目标点坐标
//       origin_x, origin_y - 坐标系原点坐标
// 返回：角度值（单位：度，范围0-360）
float xyToPolarAngle(float target_x, float target_y, float origin_x, float origin_y) {
    // 计算相对坐标
    float rel_x = target_x - origin_x;
    float rel_y = target_y - origin_y;
    
    // 使用atan2计算角度（弧度）
    float angle_rad = atan2(rel_y, rel_x);
    
    // 转换为度数
    float angle_deg = angle_rad * 180.0f / PI;
    
    // 将角度范围调整为0-360度
    if (angle_deg < 0) {
        angle_deg += 360.0f;
    }
    
    return angle_deg;
}

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

#if 0
        //从转盘抓取，需要通过视觉识别抓取物料，因为转盘不断转动，故需要等待物料停止再抓取或者实时跟踪
        int x, y;
        int arm_x, arm_y;
        int startTime = 0;
        const int traceHeightOffset = 60;
        const int turntableHeight = 80; // 转盘高度
        float y_offset ;
        float x_offset ;
        float servo0_angle;
        for (int i = 0; i < 3; i++)//循环3次，分别抓取3种颜色物料
        {
            DEBUG_LOG("正在抓取第%d种颜色物料", taskcode[i]);
            
            armControl_xyz(ttDetect.x, ttDetect.y, ttDetect.z, 1000, 100, 100); // 停在转盘中心，xyz的单位为mm
            arm_setClaw(1); // 张开夹爪以便检测物料
            startTime = millis();
            // 以上一次的xy为基础叠加摄像头数据进行跟踪，若进入死区持续一定时间或超时则退出循环执行抓取
            int follow_x = ttDetect.x;
            int follow_y = ttDetect.y;
            const float scale = 0.2f;
            const int deadzone = 20; // 死区阈值，单位与x/y一致
            const int deadzone_time = 2000; // 死区持续时间ms
            int deadzone_start = 0;
            bool in_deadzone = false;
            int cycleTimes=0;
            while (1)
            {
                cycleTimes++;
                if (millis() - startTime > VISION_GET_MATERIAL_TIMEOUT)
                {
                    errorHandle(ERROR_MATERIAL_RECOGNITION_FAILED);
                    break;
                }
                visionGetMaterial(taskcode[i], &x, &y); // 视觉识别，x,y为物料在摄像头中的坐标
                float theta0 = 90.0f-servo0.queryAngle();
                follow_x += x * scale * sin(theta0*DEG_TO_RAD) + y * scale * cos(theta0*DEG_TO_RAD);
                follow_y += -x * scale * cos(theta0*DEG_TO_RAD) + y * scale * sin(theta0*DEG_TO_RAD);
                armControl_xyz(follow_x, follow_y,
                    cycleTimes<3?ttDetect.z: turntableHeight + traceHeightOffset, 
                    cycleTimes <3?500:100, 
                    cycleTimes <3?100:50, 
                    cycleTimes <3?100:50);

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
            //摄像头和夹爪中心点不同，通过三角函数计算偏移量以使夹爪中心点对准物料中心点
            servo0_angle = servo0.queryAngle();
            sincosf(servo0_angle * DEG_TO_RAD, &x_offset, &y_offset);
            x_offset = x_offset * (-ARM_MATERIAL_OFFSET_W);
            y_offset = y_offset * (-ARM_MATERIAL_OFFSET_W);
            //移动到物料上方并抓取
            armControl_xyz(follow_x + x_offset, follow_y + y_offset, turntableHeight+10.0, 200, 100, 100);
            arm_setClaw(0); // 闭合夹爪
            waitArm();
            
            arm_x = ttDetect.x + x * scale + x_offset;
            arm_y = ttDetect.y + y * scale + y_offset;
            if (arm_x < 280)
            {
                //armControl_xyz(arm_x , arm_y , turntableHeight + 100, 500, 100, 100);
                armControl_xyz( 85 , 0 , 180 , 600, 150, 150);
                servo0.setAngle(plateAngle[taskcode[i] - 1], 500, 250, 250);
                waitArm();
                armControl_xyz(platePos[taskcode[i] - 1].x, platePos[taskcode[i] - 1].y, platePos[taskcode[i] - 1].z, 500, 100, 100);
                arm_setClaw(1);
                waitArm();
                armControl_xyz(platePos[taskcode[i] - 1].x, platePos[taskcode[i] - 1].y, platePos[taskcode[i] - 1].z + 100, 500, 100, 100);
                servo0.setAngle(90.0f, 500, 250, 250);
                waitArm();
            }
            else
            {
                //armControl_xyz(ttDetect.x, ttDetect.y, turntableHeight, 1000, 100, 100);
                //armControl_xyz(arm_x , arm_y , turntableHeight + 100, 1000, 100, 100);
                armControl_xyz( 85 , 0 , 180 , 600, 150, 150);
                servo0.setAngle(plateAngle[taskcode[i] - 1], 500, 250, 250);
                waitArm();
                armControl_xyz(platePos[taskcode[i] - 1].x, platePos[taskcode[i] - 1].y, platePos[taskcode[i] - 1].z, 1000, 100, 100);
                arm_setClaw(1);
                waitArm();
                armControl_xyz(platePos[taskcode[i] - 1].x, platePos[taskcode[i] - 1].y, platePos[taskcode[i] - 1].z + 100, 1000, 100, 100);
                servo0.setAngle(90.0f, 500, 250, 250);
                waitArm();
            }
        } // 关闭 for 循环
#else
        int posOfMaterial[3];//0,1,2号代表R,G,B，对应数字代表turntablePos中的索引
        /*
        上一次不转&&当前不转：停了一会，只能抓一个
        上一次转&&当前不转：刚停，直接抓三个
        */
        int isLastRotate = -1; //标记上一次检测时是否在转动
        int isRotate = -1; //标记当前是否在转动
        int gotMaterialCount=0;
        int ttCenterX,ttCenterY;
        // 循环计数器，防止无限循环
        int loopCounter = 0;
        const int MAX_LOOP_COUNT = 50;
        int stableCount = 0; // 连续稳定次数计数
        const int REQUIRED_STABLE_COUNT = 3; // 需要连续3次稳定才认为停止
        const float STABLE_THRESHOLD = 2.0; // 角度变化小于2度认为稳定
        
        while (1)
        {
            arm_setClaw(1);
            //armControl_raw(ttDetect, 400, 200, 200, true); // 停在转盘中心，xyz的单位为mm
            
            if(isRotate==-1||isLastRotate==-1) visionGetTurntable(&ttCenterX, &ttCenterY);
            
            int cx,cy;//当前x,y和上一次x,y
            float cAngle,lAngle;
            
            visionGetMaterial(taskcode[gotMaterialCount], &cx, &cy);
            lAngle = xyToPolarAngle(cx, cy, ttCenterX, ttCenterY);
 
            delay(200);
            visionGetMaterial(taskcode[gotMaterialCount], &cx, &cy);
            cAngle = xyToPolarAngle(cx, cy, ttCenterX, ttCenterY);
            
            // 计算角度变化（保持方向性）
            float angleDiff = lAngle - cAngle;
            UDP_LOG("物料%d last角度：%f, 当前角度：%f, 角度差：%f \n", taskcode[gotMaterialCount], lAngle, cAngle, angleDiff);
            
            // 基于转盘固定转动方向的检测逻辑
            if (angleDiff > STABLE_THRESHOLD) // 转盘顺时针转动
            {
                if (isRotate != 1) {
                    isLastRotate = isRotate; // 只在状态改变时更新
                    UDP_LOG("状态变化：停止->转动，isLastRotate=%d", isLastRotate);
                }
                isRotate = 1;
                stableCount = 0; // 重置稳定计数
            }
            else
            {
                if (isRotate != 0) {
                    isLastRotate = isRotate; // 只在状态改变时更新
                    UDP_LOG("状态变化：转动->停止，isLastRotate=%d", isLastRotate);
                }
                isRotate = 0;
                stableCount++; // 增加稳定计数
            }
            
            UDP_LOG("当前状态：isRotate=%d, isLastRotate=%d, stableCount=%d", isRotate, isLastRotate, stableCount);

            // 只有从转动状态刚停止时才开始解析物料位置
            if (isRotate == 0 && stableCount >= REQUIRED_STABLE_COUNT && isLastRotate == 1) // 解析物料位置
            {
                // 使用cx cy坐标判断物料位置：cy<0为位置0，cy>0且cx<0为位置1，cy>0且cx>0为位置2
                if (cy < 0) // cy<0为位置0
                {
                    posOfMaterial[taskcode[gotMaterialCount]-1] = 0;
                }
                else if (cy > 0 && cx < 0) // cy>0且cx<0为位置1
                {
                    posOfMaterial[taskcode[gotMaterialCount]-1] = 1;
                }
                else if (cy > 0 && cx > 0) // cy>0且cx>0为位置2
                {
                    posOfMaterial[taskcode[gotMaterialCount]-1] = 2;
                }
                else
                {
                    // 不在预期位置则继续循环
                    continue;
                }
                /*
                if(gotMaterialCount+1<3)
                {
                visionGetMaterial(taskcode[gotMaterialCount+1], &cx, &cy);
                if(cy<0)
                {
                    posOfMaterial[taskcode[gotMaterialCount+1]-1] = 0;
                }
                else
                {
                    if(cx>0)
                    {
                        posOfMaterial[taskcode[gotMaterialCount+1]-1] = 1;
                    }
                    else
                    {
                        posOfMaterial[taskcode[gotMaterialCount+1]-1] = 2;
                    }
                }
                }
                if(gotMaterialCount==0) posOfMaterial[2] = 3 - posOfMaterial[0] - posOfMaterial[1];
*/
                if(0/*isLastRotate==1*/){
                    if(gotMaterialCount==0){
                        catchFromTurntable(0, {posOfMaterial[taskcode[0]-1], taskcode[0]-1});
                        catchFromTurntable(1, {posOfMaterial[taskcode[1]-1], taskcode[1]-1});
                        catchFromTurntable(2, {posOfMaterial[taskcode[2]-1], taskcode[2]-1});
                    }
                    else if(gotMaterialCount==1){
                        catchFromTurntable(0, {posOfMaterial[taskcode[1]-1], taskcode[1]-1});
                        catchFromTurntable(2, {posOfMaterial[taskcode[2]-1], taskcode[2]-1});
                    }
                    else if(gotMaterialCount==2){
                        catchFromTurntable(0, {posOfMaterial[taskcode[2]-1], taskcode[2]-1});
                    }
                    break;
                }else{
                    if(posOfMaterial[taskcode[gotMaterialCount]-1]==-1) continue;
                    catchFromTurntable(gotMaterialCount==2?2:0, {posOfMaterial[taskcode[gotMaterialCount]-1], taskcode[gotMaterialCount]-1});
                    UDP_LOG("抓取物料%d,角度：%f,位置：%d", taskcode[gotMaterialCount]-1, cAngle, posOfMaterial[taskcode[gotMaterialCount]-1]);
                    gotMaterialCount++;
                    // 重置状态，准备检测下一个物料
                    for(int i = 0; i < 3; i++) {
                        posOfMaterial[i] = -1; // 使用-1表示未初始化
                    }
                    isRotate = -1;
                    isLastRotate = -1;
                    cAngle=-1;
                    lAngle=-1;
                    stableCount = 0;
                    if(gotMaterialCount >= 3) break; // 防止无限循环
                }
            }
            
            // 循环计数器检查
            loopCounter++;
            if (loopCounter >= MAX_LOOP_COUNT) {
                UDP_LOG("循环超过%d次，强制退出", MAX_LOOP_COUNT);
                break;
            }
            
            delay(100);
        }
#endif
        GOGOGO();

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
    // 挂起主流程任务
    if (xTaskHandleMainSequence != NULL) {
        vTaskSuspend(xTaskHandleMainSequence);
        DEBUG_LOG("主流程任务已挂起");
    }
}
void arm_groundDetect(int higherOrLower)
{
    arm_setClaw(1);
    servo0.setAngle(0.0f, 400, 200, 200);
    delay(350);
    if(higherOrLower==1)
    {
        armControl_xyz(gdDetect.x, gdDetect.y, gdDetect.z+40, 500, 250, 250);
    }
    else if(higherOrLower==2)
    {
        armControl_xyz(gdDetect.x, gdDetect.y, gdDetect.z-40, 500, 250, 250);
    }
    else {
        armControl_xyz(gdDetect.x, gdDetect.y, gdDetect.z, 500, 250, 250);
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
    
    // 获取当前位置判断是粗加工区还是暂存区
    global_position_t current_pos;
    getGlobalPosition(&current_pos);
    // 修复区域判断逻辑：基于当前位置相对于基础区域位置来判断，与caliCircle逻辑保持一致
    float dist_to_rough_base = sqrt(pow(current_pos.x - pos[3].x, 2) + pow(current_pos.y - pos[3].y, 2));
    // 如果距离粗加工区基础位置在CIRCLE_DISTANCE范围内，认为是粗加工区
    params->isRoughArea = (dist_to_rough_base <= (CIRCLE_DISTANCE + 50));
    
    // 使用lambda表达式创建任务
    auto taskFunction = [](void* parameter) -> void {
        ArmTaskParams* params = (ArmTaskParams*)parameter;
        int* taskcode = params->taskcode;
        
        /*放置到地面，需要通过视觉识别放置位置，然后在预先设置的位置基础上叠加偏移量进行放置*/
        // 区域判断：params->isRoughArea 为 true 表示粗加工区，false 表示暂存区
        if (params->isRoughArea) {
            DEBUG_LOG("当前在粗加工区执行 arm_putToGround");
        } else {
            DEBUG_LOG("当前在暂存区执行 arm_putToGround");
        }
        
        for (int i = 0; i < 3; i++)
        {
            if (taskcode[i] == 1)
            {
                if (params->isRoughArea)
                {
                    moveTo({pos[3].x, pos[3].y + CIRCLE_DISTANCE, pos[3].yaw});
                }
                else
                {
                    moveTo({pos[5].x + CIRCLE_DISTANCE, pos[5].y, pos[5].yaw});
                }
            }
            else if (taskcode[i] == 2)
            {
                if (params->isRoughArea)
                {
                    moveTo({pos[3].x, pos[3].y, pos[3].yaw});
                }
                else
                {
                    moveTo({pos[5].x, pos[5].y, pos[5].yaw});
                }
            }
            else if (taskcode[i] == 3)
            {
                if (params->isRoughArea)
                {
                    moveTo({pos[3].x, pos[3].y - CIRCLE_DISTANCE, pos[3].yaw});
                }
                else
                {
                    moveTo({pos[5].x - CIRCLE_DISTANCE, pos[5].y, pos[5].yaw});
                }
            }
            delay(200);
            if (i == 0)
            {
                armControl_xyz(0, 85, 180, 300, 150, 150, false);
                delay(250);
            }
            arm_setClaw(1);
            servo0.setAngle(plateAngle[taskcode[i] - 1], taskcode[i] == 1 ? 500 : 300, 150, 150);
            delay(taskcode[i] == 1 ? 450 : 250);
            armControl_xyz(platePos[taskcode[i] - 1].x, platePos[taskcode[i] - 1].y, platePos[taskcode[i] - 1].z, 300, 150, 150);
            arm_setClaw(0);
            waitArm();
            armControl_xyz(platePos[taskcode[i] - 1].x, platePos[taskcode[i] - 1].y, platePos[taskcode[i] - 1].z + 100, 400, 200, 200, false);
            delay(350);
            servo0.setAngle(0.0f, taskcode[i] == 1 ? 500 : 300, 150, 150);
            delay(taskcode[i] == 1 ? 450 : 250);
            waitArrived();
            delay(200);
            armControl_xyz(0, 213, 0, 800, 400, 400);
            delay(200);
            arm_setClaw(1);
            waitArm();
            armControl_xyz(0, 85, 180, 500, 250, 250, false);

#if 0
            arm_setClaw(1);
            armControl_xyz(platePos[taskcode[i]-1].x, platePos[taskcode[i]-1].y, platePos[taskcode[i]-1].z + overPlateHeight, 400, 200, 200); // 停在托盘上方
            armControl_xyz(platePos[taskcode[i]-1].x, platePos[taskcode[i]-1].y, platePos[taskcode[i]-1].z,400, 200, 200); // 下降到托盘
            arm_setClaw(0); // 闭合夹爪
            waitArm();
            armControl_xyz(platePos[taskcode[i]-1].x, platePos[taskcode[i]-1].y, platePos[taskcode[i]-1].z + overPlateHeight, 400, 200, 200); // 上升到托盘上方
            for (int j = 0; j < 5; j++)
            {
                armControl_xyz(keyPos[taskcode[i]-1][j].x, keyPos[taskcode[i]-1][j].y, keyPos[taskcode[i]-1][j].z, 1000, 500, 500, false); // 从托盘到色环的关键点
            };
            waitArm();
            armControl_xyz(circlePos[taskcode[i]-1].x, circlePos[taskcode[i]-1].y, circlePos[taskcode[i]-1].z + overCircleHeight, 400, 200, 200); // 到色环上方

            /*获取色环偏移量并叠加偏移量*/
            int x, y;
            const int deadzone = 10; // 死区阈值
            const int timeout_ms = 3000; // 超时时间
            int start_time = millis();

            float theta0;
            while (millis() - start_time < timeout_ms) {
                if (visionGetCircle(&x, &y)) {
                    // 检查是否在死区内
                    if (abs(x) < deadzone && abs(y) < deadzone) {
                        DEBUG_LOG("色环校准完成，误差在死区内: x=%d, y=%d", x, y);
                        break;
                    }
                    
                    // 计算偏移量并应用
                    theta0 = -servo0.queryAngle();
                    float offset_x = (x*cos(theta0*DEG_TO_RAD) - y*sin(theta0*DEG_TO_RAD))*scale;
                    float offset_y = (x*sin(theta0*DEG_TO_RAD) + y*cos(theta0*DEG_TO_RAD))*scale;
                    
                    circlePos[taskcode[i]-1].x += offset_x;
                    circlePos[taskcode[i]-1].y += offset_y;
                    
                    // 移动到新位置
                    armControl_xyz(circlePos[taskcode[i]-1].x, circlePos[taskcode[i]-1].y, circlePos[taskcode[i]-1].z + overCircleHeight, 400, 200, 200);
                    waitArm();
                } else {
                    DEBUG_LOG("获取色环位置失败，继续尝试");
                    delay(100);
                }
            }
            
            if (millis() - start_time >= timeout_ms) {
                DEBUG_LOG("色环校准超时");
            }

            float x_offset = ARM_MATERIAL_OFFSET_W * sin(theta0*DEG_TO_RAD) - ARM_MATERIAL_OFFSET_X * cos(theta0*DEG_TO_RAD);
            float y_offset = ARM_MATERIAL_OFFSET_W * cos(theta0*DEG_TO_RAD) + ARM_MATERIAL_OFFSET_X * sin(theta0*DEG_TO_RAD);
            circlePos[taskcode[i]-1].x -= x_offset;
            circlePos[taskcode[i]-1].y -= y_offset;
    
            armControl_xyz(circlePos[taskcode[i]-1].x, circlePos[taskcode[i]-1].y, circlePos[taskcode[i]-1].z, 400, 200, 200); // 放到色环上
            arm_setClaw(1);
            waitArm();
            armControl_xyz(circlePos[taskcode[i]-1].x, circlePos[taskcode[i]-1].y, circlePos[taskcode[i]-1].z + overCircleHeight, 400, 200, 200); // 上升到色环上方
            armControl_xyz(0, 93, 130, 400, 200, 200);
#endif
        }
        if(!params->isRoughArea)
        {
            delay(250);
            armControl_raw(ttDetect, 800, 400, 400, false); // 停在转盘中心，xyz的单位为mm
        }
        GOGOGO();

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
    // 挂起主流程任务
    if (xTaskHandleMainSequence != NULL) {
        vTaskSuspend(xTaskHandleMainSequence);
        DEBUG_LOG("主流程任务已挂起");
    }
}

void arm_catchFromGround(int taskcode[3], int cycleNum)//将物料从地面抓取到托盘
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
    params->cycleNum = cycleNum;
    
    // 使用lambda表达式创建任务
    auto taskFunction = [](void* parameter) -> void {
        ArmTaskParams* params = (ArmTaskParams*)parameter;
        int* taskcode = params->taskcode;
        int cycleNum = params->cycleNum;

        if (taskcode[0] == 1)
        {
            moveTo({pos[3].x, pos[3].y + CIRCLE_DISTANCE, pos[3].yaw});
        }
        else if (taskcode[0] == 2)
        {
            moveTo({pos[3].x, pos[3].y, pos[3].yaw});
        }
        else if (taskcode[0] == 3)
        {
            moveTo({pos[3].x, pos[3].y - CIRCLE_DISTANCE, pos[3].yaw});
        }
        delay(500);
        /*从地面抓取，获取放置时的偏差，叠加偏移量进行抓取*/
        for (int i = 0; i < 3; i++)// 循环3次，分别抓取3种颜色物料
        {
            if (i == 0)
            {
                armControl_xyz(0, 85, 180, 300, 150, 150, false);
            }
            arm_setClaw(1);
            waitArrived();
            delay(200);
            armControl_xyz(0, 213, 0, 800, 400, 400);
            delay(200);
            arm_setClaw(0);
            waitArm();
            delay(200);
            armControl_xyz(0, 85, 180, 500, 250, 250, false);
            delay(350);
            if(i<2){
            if (taskcode[i+1] == 1)
            {
                moveTo({pos[3].x, pos[3].y + CIRCLE_DISTANCE, pos[3].yaw});
            }
            else if (taskcode[i+1] == 2)
            {
                moveTo({pos[3].x, pos[3].y, pos[3].yaw});
            }
            else if (taskcode[i+1] == 3)
            {
                moveTo({pos[3].x, pos[3].y - CIRCLE_DISTANCE, pos[3].yaw});
            }
            }
            servo0.setAngle(plateAngle[taskcode[i] - 1], taskcode[i] == 1 ? 500 : 400, 200, 200);
            delay(taskcode[i] == 1 ? 450 : 350);
            armControl_xyz(platePos[taskcode[i] - 1].x, platePos[taskcode[i] - 1].y, platePos[taskcode[i] - 1].z, 300, 150, 150);
            arm_setClaw(1);
            waitArm();
            if(i==2){
                GOGOGO();
            }
            armControl_xyz(platePos[taskcode[i] - 1].x, platePos[taskcode[i] - 1].y, platePos[taskcode[i] - 1].z + 100, 400, 200, 200, false);
            delay(350);
            servo0.setAngle(0.0f, taskcode[i] == 1 ? 500 : 300, 150, 150);
            delay(taskcode[i] == 1 ? 450 : 250);
            armControl_xyz(0, 85, 180, 300, 150, 150, false);
            delay(250);
#if 0
            arm_setClaw(1);
            armControl_xyz(circlePos[taskcode[i]-1].x, circlePos[taskcode[i]-1].y, circlePos[taskcode[i]-1].z + overCircleHeight, 400, 200, 200); // 上升到色环上方
            armControl_xyz(circlePos[taskcode[i]-1].x, circlePos[taskcode[i]-1].y, circlePos[taskcode[i]-1].z, 400, 200, 200); // 下降到色环
            arm_setClaw(0); // 闭合夹爪
            waitArm();
            armControl_xyz(circlePos[taskcode[i]-1].x, circlePos[taskcode[i]-1].y, circlePos[taskcode[i]-1].z + overCircleHeight, 400, 200, 200); // 上升到色环上方
            for (int j = 0; j < 5; j++)// 循环5次，分别从色环到托盘的关键点
            {                                                                                                                             
                armControl_xyz(keyPos[taskcode[i] + 3-1][j].x, keyPos[taskcode[i] + 3-1][j].y, keyPos[taskcode[i] + 3-1][j].z, 1000, 500, 500, false); // 从色环到托盘的关键点
            };
            waitArm();
            //armControl_xyz(platePos[taskcode[i]-1].x, platePos[taskcode[i]-1].y, platePos[taskcode[i]-1].z + overPlateHeight, 400, 200, 200); // 到托盘上方
            armControl_xyz(platePos[taskcode[i]-1].x, platePos[taskcode[i]-1].y, platePos[taskcode[i]-1].z, 400, 200, 200); // 放到托盘上
            arm_setClaw(1);
            waitArm();
            armControl_xyz(platePos[taskcode[i]-1].x, platePos[taskcode[i]-1].y, platePos[taskcode[i]-1].z + overPlateHeight, 400, 200, 200); // 上升到托盘上方
            armControl_xyz(0, 93, 130, 400, 200, 200);
#endif
        }
        arm_groundDetect(cycleNum == 1 ? 0 : 1);

        GOGOGO();

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
    // 挂起主流程任务
    if (xTaskHandleMainSequence != NULL) {
        vTaskSuspend(xTaskHandleMainSequence);
        DEBUG_LOG("主流程任务已挂起");
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
            if (taskcode[i] == 1)
            {
                moveTo({pos[5].x + CIRCLE_DISTANCE, pos[5].y, pos[5].yaw});
            }
            else if (taskcode[i] == 2)
            {
                moveTo({pos[5].x, pos[5].y, pos[5].yaw});
            }
            else if (taskcode[i] == 3)
            {
                moveTo({pos[5].x - CIRCLE_DISTANCE, pos[5].y, pos[5].yaw});
            }
            delay(500);
            if (i == 0)
            {
                armControl_xyz(0, 85, 180, 300, 150, 150, false);
                delay(250);
            }
            arm_setClaw(1);
            servo0.setAngle(plateAngle[taskcode[i] - 1], taskcode[i] == 1 ? 500 : 300, 150, 150);
            delay(taskcode[i] == 1 ? 450 : 250);
            armControl_xyz(platePos[taskcode[i] - 1].x, platePos[taskcode[i] - 1].y, platePos[taskcode[i] - 1].z, 300, 150, 150);
            arm_setClaw(0);
            waitArm();
            armControl_xyz(platePos[taskcode[i] - 1].x, platePos[taskcode[i] - 1].y, platePos[taskcode[i] - 1].z + 100, 400, 200, 200, false);
            delay(350);
            servo0.setAngle(0.0f, taskcode[i] == 1 ? 500 : 300, 150, 150);
            delay(taskcode[i] == 1 ? 450 : 250);
            armControl_xyz(0, 200, MATERIAL_HEIGHT+10, 800, 400, 400);
            arm_setClaw(1);
            armControl_xyz(0, 85, 180, 300, 150, 150, false);
            //delay(250);
#if 0
            arm_setClaw(1);
            armControl_xyz(platePos[taskcode[i]-1].x, platePos[taskcode[i]-1].y, platePos[taskcode[i]-1].z + overPlateHeight, 400, 200, 200); // 停在托盘上方
            armControl_xyz(platePos[taskcode[i]-1].x, platePos[taskcode[i]-1].y, platePos[taskcode[i]-1].z, 400, 200, 200); // 下降到托盘
            arm_setClaw(0); // 闭合夹爪
            waitArm();
            armControl_xyz(platePos[taskcode[i]-1].x, platePos[taskcode[i]-1].y, platePos[taskcode[i]-1].z + overPlateHeight, 400, 200, 200); // 上升到托盘上方
            for (int j = 0; j < 5; j++)
            {
                armControl_xyz(keyPos[taskcode[i]+6-1][j].x, keyPos[taskcode[i]+6-1][j].y, keyPos[taskcode[i]+6-1][j].z, 400, 200, 200, false); // 从托盘到色环的关键点
            };
            waitArm();
            //armControl_xyz(circlePos[taskcode[i]-1].x, circlePos[taskcode[i]-1].y, circlePos[taskcode[i]-1].z + MATERIAL_HEIGHT + overCircleHeight, 400, 200, 200); // 到色环上方

            /*获取色环偏移量并叠加偏移量*/
            int x, y;
            visionGetCircle(&x, &y);
            circlePos[taskcode[i]].x += x*scale;
            circlePos[taskcode[i]].y += y*scale;

            armControl_xyz(circlePos[taskcode[i]-1].x, circlePos[taskcode[i]-1].y, circlePos[taskcode[i]-1].z + MATERIAL_HEIGHT, 400, 200, 200); // 放到色环的物料上
            arm_setClaw(1);
            waitArm();
            armControl_xyz(circlePos[taskcode[i]-1].x, circlePos[taskcode[i]-1].y, circlePos[taskcode[i]-1].z + MATERIAL_HEIGHT +  overCircleHeight, 400, 200, 200); // 上升到色环上方
            armControl_xyz(0, 93, 140, 400, 200, 200);
#endif
        }

        GOGOGO();

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
    // 挂起主流程任务
    if (xTaskHandleMainSequence != NULL) {
        vTaskSuspend(xTaskHandleMainSequence);
        DEBUG_LOG("主流程任务已挂起");
    }
}
