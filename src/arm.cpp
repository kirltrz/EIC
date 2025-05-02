#include "arm.h"
#include "config.h"

FSUS_Protocol protocol(&SERVO_SERIAL, SERIAL_BAUDRATE);
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
const armPos fold = {0, 0, 0};        // 折叠状态
const armPos ttDetect = {0, 0, 0};    // 转盘检测位置
const armPos rPlate = {0, 0, 0};      // 红色物料托盘位置
const armPos bPlate = {0, 0, 0};      // 蓝色物料托盘位置
const armPos gPlate = {0, 0, 0};      // 绿色物料托盘位置
const armPos gdDetect = {0, 0, 0};    // 地面检测位置
const armPos rCircleBase = {0, 0, 0}; // 红色色环基础位置（未视觉纠偏的位置）
const armPos bCircleBase = {0, 0, 0}; // 蓝色色环基础位置（未视觉纠偏的位置）
const armPos gCircleBase = {0, 0, 0}; // 绿色色环基础位置（未视觉纠偏的位置）

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
    /*舵机角度限幅 限幅角度待定 宏定义*/
    servo0.setAngleRange(0, 0); // 云台舵机
    servo1.setAngleRange(0, 0); // 一级关节舵机
    servo2.setAngleRange(0, 0); // 二级关节舵机
    servo3.setAngleRange(0, 0); // 三级关节舵机
    servo4.setAngleRange(0, 0); // 夹爪舵机
}
/*逆解算 根据下x,y,z坐标解算出具体关节的舵机角度*/
void armCalculate_inverse(float x, float y, float z, float *out_arm_degree)
{
    float shortSide;        // 短边长度
    float hypotenuse;       // 斜边长度
    float first_arm_degree, // 大臂角度
        second_arm_degree,  // 小臂角度
        first_arm_rad_1,    // 大臂弧度计算中间值1
        first_arm_rad_2;    // 大臂弧度计算中间值2
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
    /* 机械臂yoz平面逆解算 */

    // 考虑爪子长度高度,修正坐标系
    y = y - ARM_THIRD_LENGTH;
    z = z + ARM_MATERIAL_HEIGHT;
    if (y < 0)
        y = 0;

    // 计算斜边
    shortSide = ARM_FIRST_JOINT_HEIGHT - z;
    if (shortSide < 0)
        shortSide = shortSide * -1.0f;
    hypotenuse = sqrt(y * y + shortSide * shortSide); // 斜边长度

    // 计算大臂角度
    first_arm_rad_1 = acos(((ARM_SECOND_LENGTH * ARM_SECOND_LENGTH) -
                            (ARM_FIRST_LENGTH * ARM_FIRST_LENGTH) +
                            hypotenuse * hypotenuse) /
                           (2.0f * ARM_SECOND_LENGTH * hypotenuse));

    // 计算大臂中间值2弧度

    if (ARM_FIRST_JOINT_HEIGHT > z)
        first_arm_rad_2 = atan(y / shortSide) - PI / 2.0f;
    else if (ARM_FIRST_JOINT_HEIGHT < z)
        first_arm_rad_2 = atan(shortSide / y);
    else
        first_arm_rad_2 = 0.0f;

    // 计算大臂角度
    first_arm_degree = (first_arm_rad_1 + first_arm_rad_2) * 180.0f / PI; // 大臂角度(度)

    // 计算小臂角度
    second_arm_degree = acos(((ARM_SECOND_LENGTH * ARM_SECOND_LENGTH) +
                              (ARM_FIRST_LENGTH * ARM_FIRST_LENGTH) -
                              hypotenuse * hypotenuse) /
                             (2.0f * ARM_SECOND_LENGTH * ARM_SECOND_LENGTH)) *
                        180.0f / PI; // 小臂角度(度)

    first_arm_degree = 90.0f - first_arm_degree;                       // 将大臂角转换为相对于竖直方向的角度 ，向前为正值
    second_arm_degree = -90.0f + second_arm_degree - first_arm_degree; // 将小臂角转换为相对于水平线的角度，低于水平线为负值
    /* 机械臂xy平面逆解算 */
    float theta0 = atan2(y, x) * 180.0 / PI; // 云台舵机角度(度)

    out_arm_degree[0] = first_arm_degree;  // 大臂角度
    out_arm_degree[1] = second_arm_degree; // 小臂角度(度)
    out_arm_degree[2] = theta0;            // 云台舵机角度(度)
}
/**
 * 根据大小臂角度计算末端执行器的位置坐标
 *@brief 机械臂运动学正解算
 *@param first_arm_degree: 大臂角度,竖直方向为0,向前为正角度
 *@param second_arm_degree: 小臂角度,水平方向为0,向上为正角度
 *@param theta0   云台舵机角度(度)
 *@param out_arm_location_yz[0]: 输出坐标y
 *@param out_arm_location_yz[1]: 输出坐标z
 *@param out_arm_location_yz[1]: 输出坐标x
 */
float *armCalculate_forward(float first_arm_degree, float second_arm_degree, float theta0, float *out_arm_location_yz)
{
    float z1, z2, y1, y2;                                        // 大臂和小臂的z轴和y轴增量
    float x1, x2;                                                // 大臂和小臂的x轴增量
    z1 = ARM_SECOND_LENGTH * cos(first_arm_degree * PI / 180.0); // 大臂z轴增量
    y1 = ARM_SECOND_LENGTH * sin(first_arm_degree * PI / 180.0); // 大臂y轴增量

    z2 = ARM_FIRST_LENGTH * sin(second_arm_degree * PI / 180.0);
    y2 = ARM_FIRST_LENGTH * cos(second_arm_degree * PI / 180.0);

    out_arm_location_yz[0] = y1 + y2 + ARM_THIRD_LENGTH;                          // 末端执行器y坐标
    out_arm_location_yz[2] = out_arm_location_yz[0] * tan(theta0 * PI / 180.0);   // 末端执行器x坐标
    out_arm_location_yz[1] = z1 + z2 - ARM_THIRD_LENGTH + ARM_FIRST_JOINT_HEIGHT; // 末端执行器z坐标

    return out_arm_location_yz;
}
/**
 *@brief 机械大小臂角度控制
 *@param first_arm_degree: 大臂相对角度
 *@param second_arm_degree: 小臂相对角度
 *@return 无
 */
void armSet_position(float first_arm_degree, float second_arm_degree, float theta0)
{
    // 角度限制
    if (theta0 < BASIS_ARM_ANGLE_MIN)
        theta0 = BASIS_ARM_ANGLE_MIN;
    if (theta0 > BASIS_ARM_ANGLE_MAX)
        theta0 = BASIS_ARM_ANGLE_MAX;
    if (first_arm_degree < FIRST_ARM_ANGLE_MIN)
        first_arm_degree = FIRST_ARM_ANGLE_MIN;
    if (first_arm_degree > FIRST_ARM_ANGLE_MAX)
        first_arm_degree = FIRST_ARM_ANGLE_MAX;
    if (second_arm_degree < SECOND_ARM_ANGLE_MIN)
        second_arm_degree = SECOND_ARM_ANGLE_MIN;
    if (second_arm_degree > SECOND_ARM_ANGLE_MAX)
        second_arm_degree = SECOND_ARM_ANGLE_MAX;

    // 设置舵机角度
    servo0.setAngle(theta0);            // 云台舵机
    servo1.setAngle(first_arm_degree);  // 一级关节舵机
    servo2.setAngle(second_arm_degree); // 二级关节舵机
}
/**
 *@brief 机械臂xyz平面位置控制
 *@param x: x坐标,单位mm
 *@param y: y坐标,单位mm
 *@param z: z轴坐标,单位mm
 *@return void
 */
void armControl_xyz(float x, float y, float z)
{
    float xyz_to_angle[3];

    armCalculate_inverse(x, y, z, xyz_to_angle);
    armSet_position(xyz_to_angle[0], xyz_to_angle[1], xyz_to_angle[2]);
}

void arm_ScanQRcode()
{
    /*扫描二维码，与控制xyz不同，机械臂前端需抬起使摄像头朝向二维码，无需处理视觉部分*/
}

void arm_catchFromTurntable(int taskcode[3])
{
    /*从转盘抓取，需要通过视觉识别抓取物料，因为转盘不断转动，故需要等待物料停止再抓取或者实时跟踪*/
    const int turntableHeight = 80; // 转盘高度
}

void arm_putToGround(int taskcode[3], int circleOffset[3][2] /*传出当前色环偏移量*/)
{
    /*放置到地面，需要通过视觉识别放置位置，然后在预先设置的位置基础上叠加偏移量进行放置*/
}

void arm_catchFromGround(int taskcode[3], const int circleOffset[3][2] /*传入当前色环偏移量*/)
{
    /*从地面抓取，获取放置时的偏差，叠加偏移量进行抓取*/
}

void arm_putToMaterial(int taskcode[3])
{
    /*码垛，同样需要通过视觉识别物料位置，然后在预先设置的位置基础上叠加偏移量进行放置*/
}
