#include "motion.h"
#include "sensor.h"
#include "taskManager.h" // 导入wait函数

// 声明ZDT_MOTOR控制对象指针
ZDTX42V2* motor = nullptr;

// 定义电机控制参数
// 注意：DEFAULT_ACC已在motion.h中定义为100

// 定义电机地址
#define MOTOR_FR 3  // 前右轮电机地址
#define MOTOR_FL 4  // 前左轮电机地址
#define MOTOR_BL 1  // 后左轮电机地址
#define MOTOR_BR 2  // 后右轮电机地址

// 全向轮底盘参数
#define WHEEL_RADIUS 41.0f    // 轮子半径，单位mm
#define ROBOT_RADIUS 106.5f   // 机器人中心到轮子的距离，单位mm

// PID控制参数 - 运动中使用
#define POS_KP 200.0f     // 位置环比例系数 - 极大增大
#define POS_KI 1.0f      // 位置环积分系数 - 增大
#define POS_KD 1.0f     // 位置环微分系数 - 增大
#define YAW_KP 200.0f     // 偏航角比例系数 - 极大增大
#define YAW_KI 1.0f      // 偏航角积分系数 - 增大
#define YAW_KD 1.0f     // 偏航角微分系数 - 增大

// PID控制参数 - 位置锁定时使用（比例系数更高，以增强保持力）
#define HOLD_POS_KP 50.0f    // 位置保持比例系数
#define HOLD_POS_KI 0.1f     // 完全禁用积分作用
#define HOLD_POS_KD 15.0f     // 位置保持微分系数
#define HOLD_YAW_KP 100.0f    // 偏航角保持比例系数
#define HOLD_YAW_KI 0.1f     // 完全禁用积分作用
#define HOLD_YAW_KD 15.0f     // 偏航角保持微分系数

// 运动控制参数
#define MAX_LINEAR_SPEED 5000.0f  // 最大线速度，单位mm/s - 大幅增加
#define MAX_ANGULAR_SPEED 720.0f  // 最大角速度，单位度/s - 大幅增加
#define MIN_SPEED_RPM 0.1f       // 最小速度，单位RPM - 增大最小速度
#define MAX_SPEED_RPM 2400.0f    // 最大速度，单位RPM - 恢复原始设置

// 位置保持模式参数
#define HOLD_MAX_LINEAR_SPEED 5000.0f  // 位置保持时最大线速度，单位mm/s
#define HOLD_MAX_ANGULAR_SPEED 720.0f  // 位置保持时最大角速度，单位度/s

// 电机位置累积值
static float motor_pos_fr = 0;
static float motor_pos_fl = 0;
static float motor_pos_bl = 0;
static float motor_pos_br = 0;

// 目标位置
static POS targetPos = {0, 0, 0};
// 运动状态
static bool isMoving = false;  // 正在移动到目标位置
static bool isHolding = false; // 正在保持目标位置
// PID积分项
static float pos_x_integral = 0;
static float pos_y_integral = 0;
static float yaw_integral = 0;
// PID上一次误差
static float pos_x_last_error = 0;
static float pos_y_last_error = 0;
static float yaw_last_error = 0;

void initMotor()
{
    // 初始化串口通信，用于与电机通信
    Serial2.begin(115200, SERIAL_8N1, 10 /*RX*/, 9 /*TX*/);
    delay(1000);  // 等待通信稳定
    
    Serial.println("开始初始化电机...");
    
    // 初始化电机对象
    motor = new ZDTX42V2(&Serial2);
    
    delay(500);  // 等待电机初始化完成
    
    // 确保所有电机都停止
    motor->stopNow(MOTOR_FR, 0);
    motor->stopNow(MOTOR_FL, 0);
    motor->stopNow(MOTOR_BL, 0);
    motor->stopNow(MOTOR_BR, 0);
    
    Serial.println("电机初始化和通信测试完成");
}

// 将全局坐标系中的速度转换为机器人局部坐标系中的速度
void globalToLocalVelocity(float global_vx, float global_vy, float yaw_rad, float &local_vx, float &local_vy) {
    // yaw_rad是机器人相对于全局坐标系的偏航角，单位是弧度
    // 全局坐标到局部坐标的旋转变换，交换X和Y的映射关系
    local_vx = -global_vy * cos(yaw_rad) + global_vx * sin(yaw_rad);
    local_vy = -global_vx * cos(yaw_rad) - global_vy * sin(yaw_rad);
}

// 计算四个轮子的位移增量
void calculateWheelPositions(float global_vx, float global_vy, float omega, float dt, float yaw_rad, float wheel_positions[4])
{
    // 将全局坐标系中的速度转换为机器人局部坐标系中的速度
    float local_vx, local_vy;
    globalToLocalVelocity(global_vx, global_vy, yaw_rad, local_vx, local_vy);
    
    // 计算轮子角速度 (rad/s)
    float wheel_speeds[4];
    
    // 根据轮子布局进行正确的运动学逆解算
    // 1号轮在左下角(0,0)，轮面与y轴平行 - 后左轮 (BL)
    // 2号轮在左上角(0,L)，轮面与x轴平行 - 前左轮 (FL)
    // 3号轮在右上角(L,L)，轮面与y轴平行 - 前右轮 (FR)
    // 4号轮在右下角(L,0)，轮面与x轴平行 - 后右轮 (BR)
    
    // 翻转X轴控制方向，修正左右行驶方向
    // 交换X和Y对轮子的影响关系
    wheel_speeds[0] = (local_vx - omega * ROBOT_RADIUS) / WHEEL_RADIUS; // 后左轮 (BL) - 1号
    wheel_speeds[1] = (local_vy - omega * ROBOT_RADIUS) / WHEEL_RADIUS;  // 前左轮 (FL) - 2号
    wheel_speeds[2] = (-local_vx - omega * ROBOT_RADIUS) / WHEEL_RADIUS;  // 前右轮 (FR) - 3号
    wheel_speeds[3] = (-local_vy - omega * ROBOT_RADIUS) / WHEEL_RADIUS; // 后右轮 (BR) - 4号
    
    // 将角速度(rad/s)转换为角度增量(°)
    // 角度增量 = 角速度 * 时间 * (180/π)
    for (int i = 0; i < 4; i++) {
        wheel_positions[i] = wheel_speeds[i] * dt * (180.0 / PI);
    }
}

// 限制输入值在最小和最大值之间
float constrain_float(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

void moveTo(POS pos, float speed, int acc, int dec)
{
    /*移动到目标点，仅用于传入目标点*/
    // 保存目标位置
    targetPos = pos;
    
    // 设置运动状态
    isMoving = true;
    isHolding = false;
    
    // 重置PID积分项
    pos_x_integral = 0;
    pos_y_integral = 0;
    yaw_integral = 0;
    
    // 重置PID上一次误差
    pos_x_last_error = 0;
    pos_y_last_error = 0;
    yaw_last_error = 0;
}

// 停止运动并释放电机控制
void stopMotion()
{
    if (motor != nullptr) {
        // 停止所有电机 - 使用停止命令
        motor->stopNow(MOTOR_FR, 0);
        motor->stopNow(MOTOR_FL, 0);
        motor->stopNow(MOTOR_BL, 0);
        motor->stopNow(MOTOR_BR, 0);
        
        delay(20); // 等待停止命令执行
        
        // 重置当前位置为零点
        motor_pos_fr = 0.0f;
        motor_pos_fl = 0.0f;
        motor_pos_bl = 0.0f;
        motor_pos_br = 0.0f;
        
        // 重置电机位置计数器
        motor->resetCurPosToZero(MOTOR_FR);
        motor->resetCurPosToZero(MOTOR_FL);
        motor->resetCurPosToZero(MOTOR_BL);
        motor->resetCurPosToZero(MOTOR_BR);
        
        // 输出调试信息
        Serial.println("电机已停止并重置位置");
    }
    
    isMoving = false;
    isHolding = false;
}

void moveTask(void* pvParameters)
{
    /*移动任务,不断向电机发送控制指令*/
    uint8_t rxCmd[128] = {0};
    uint8_t rxCount = 0;
    global_position_t currentPosition;
    float wheel_speeds[4] = {0}; // 改为直接存储速度而不是位置增量
    
    const int PID_UPDATE_RATE = 100; // PID更新频率，100Hz
    const int PID_INTERVAL = 1000 / PID_UPDATE_RATE; // PID更新间隔，10ms
    unsigned long lastPID_Time = 0;
    unsigned long lastCmd_Time = 0;
    
    // 任务启动时强制停止所有电机并重置状态
    stopMotion();
    
    while(1) {
        // 确保电机控制对象已初始化
        if (motor == nullptr) {
            wait(10);
            continue;
        }
        
        // 获取当前位置
        getGlobalPosition(&currentPosition);
        
        // 当前时间
        unsigned long currentTime = millis();
        
        // 高频率更新PID控制（当处于移动状态或位置保持状态时）
        if ((isMoving || isHolding) && (currentTime - lastPID_Time >= PID_INTERVAL)) {
            lastPID_Time = currentTime;
            
            // 计算误差
            float pos_x_error = targetPos.x - currentPosition.x;
            float pos_y_error = targetPos.y - currentPosition.y;
            float yaw_error = targetPos.yaw - currentPosition.continuousYaw;
            
            // 更新积分项
            pos_x_integral += pos_x_error * PID_INTERVAL / 1000.0f;
            pos_y_integral += pos_y_error * PID_INTERVAL / 1000.0f;
            yaw_integral += yaw_error * PID_INTERVAL / 1000.0f;
            
            // 限制积分项防止积分饱和
            pos_x_integral = constrain_float(pos_x_integral, -50, 50);
            pos_y_integral = constrain_float(pos_y_integral, -50, 50);
            yaw_integral = constrain_float(yaw_integral, -10, 10);
            
            // 计算微分项
            float pos_x_derivative = (pos_x_error - pos_x_last_error) * 1000.0f / PID_INTERVAL;
            float pos_y_derivative = (pos_y_error - pos_y_last_error) * 1000.0f / PID_INTERVAL;
            float yaw_derivative = (yaw_error - yaw_last_error) * 1000.0f / PID_INTERVAL;
            
            // 保存当前误差
            pos_x_last_error = pos_x_error;
            pos_y_last_error = pos_y_error;
            yaw_last_error = yaw_error;
            
            // 根据当前模式选择PID系数
            float posKp, posKi, posKd, yawKp, yawKi, yawKd;
            float max_linear_speed, max_angular_speed;
            
            if (isMoving) {
                // 运动模式PID参数
                posKp = POS_KP;
                posKi = POS_KI;
                posKd = POS_KD;
                yawKp = YAW_KP;
                yawKi = YAW_KI;
                yawKd = YAW_KD;
                max_linear_speed = MAX_LINEAR_SPEED;
                max_angular_speed = MAX_ANGULAR_SPEED;
            } else {
                // 位置保持模式PID参数（更高的比例系数以增强保持力）
                posKp = HOLD_POS_KP;
                posKi = HOLD_POS_KI;
                posKd = HOLD_POS_KD;
                yawKp = HOLD_YAW_KP;
                yawKi = HOLD_YAW_KI;
                yawKd = HOLD_YAW_KD;
                max_linear_speed = HOLD_MAX_LINEAR_SPEED;
                max_angular_speed = HOLD_MAX_ANGULAR_SPEED;
            }
            
            // 计算PID输出 - 这是全局坐标系中的速度
            float global_vx = posKp * pos_x_error + posKi * pos_x_integral + posKd * pos_x_derivative;
            float global_vy = posKp * pos_y_error + posKi * pos_y_integral + posKd * pos_y_derivative;
            float omega = yawKp * yaw_error + yawKi * yaw_integral + yawKd * yaw_derivative;
            
            // 限制最大速度
            float linear_speed = sqrt(global_vx*global_vx + global_vy*global_vy);
            if (linear_speed > max_linear_speed) {
                float scale = max_linear_speed / linear_speed;
                global_vx *= scale;
                global_vy *= scale;
            }
            
            // 限制最大角速度
            omega = constrain_float(omega, -max_angular_speed, max_angular_speed);
            
            // 将角度从度转换为弧度
            float yaw_rad = currentPosition.rawYaw * PI / 180.0f;
            
            // 计算四个轮子的速度 (直接计算RPM)
            float local_vx, local_vy;
            globalToLocalVelocity(global_vx, global_vy, yaw_rad, local_vx, local_vy);
            
            // 计算轮子角速度 (rad/s)
            wheel_speeds[0] = (local_vx - omega * PI / 180.0f * ROBOT_RADIUS) / WHEEL_RADIUS; // 后左轮 (BL) - 1号
            wheel_speeds[1] = (local_vy - omega * PI / 180.0f * ROBOT_RADIUS) / WHEEL_RADIUS; // 前左轮 (FL) - 2号
            wheel_speeds[2] = (-local_vx - omega * PI / 180.0f * ROBOT_RADIUS) / WHEEL_RADIUS; // 前右轮 (FR) - 3号
            wheel_speeds[3] = (-local_vy - omega * PI / 180.0f * ROBOT_RADIUS) / WHEEL_RADIUS; // 后右轮 (BR) - 4号
            
            // 将rad/s转换为RPM (1 rad/s = 60/(2*pi) RPM)
            for (int i = 0; i < 4; i++) {
                wheel_speeds[i] = wheel_speeds[i] * 60.0f / (2.0f * PI);
            }
            
            // 确保每10ms发送一次电机控制命令
            if (currentTime - lastCmd_Time >= 10) {
                lastCmd_Time = currentTime;
                
                // 获取每个电机的地址
                uint8_t motor_addrs[4] = {MOTOR_FR, MOTOR_FL, MOTOR_BL, MOTOR_BR};
                
                // 直接使用速度控制模式控制电机
                for (int i = 0; i < 4; i++) {
                    float speed = wheel_speeds[i];
                    
                    // 限制最小速度
                    if (abs(speed) < MIN_SPEED_RPM && abs(speed) > 0.001f) {
                        speed = (speed > 0) ? MIN_SPEED_RPM : -MIN_SPEED_RPM;
                    }
                    
                    // 如果速度几乎为零，不发送命令
                    if (abs(speed) < 0.001f) {
                        continue;
                    }
                    
                    // 限制最大速度
                    speed = constrain_float(speed, -MAX_SPEED_RPM, MAX_SPEED_RPM);
                    
                    // 确定方向和速度值
                    uint8_t dir = (speed >= 0) ? 0 : 1;  // 0为正向，1为反向
                    float abs_speed = abs(speed);
                    
                    // 使用速度控制模式
                    motor->velocityControl(motor_addrs[i], dir, DEFAULT_ACC, abs_speed, 0);
                    
                    // 接收并处理来自电机的反馈数据
                    motor->receiveData(rxCmd, &rxCount);
                    
                    // 添加调试信息，显示电机回复
                    /*if (i == 0 && (currentTime % 1000) < 10) { // 每秒输出一次第一个电机的回复情况
                        Serial.print("电机回复数据: 长度=");
                        Serial.print(rxCount);
                        Serial.print(", 数据:");
                        for(uint8_t j = 0; j < rxCount && j < 10; j++) {
                            Serial.print(" 0x");
                            Serial.print(rxCmd[j], HEX);
                        }
                        Serial.println();
                    }*/
                }
            }
            
            // 判断是否到达目标点
            if (isMoving && arrived()) {
                // 切换到位置保持模式，而不是停止电机
                isMoving = false;
                isHolding = true;
                
                // 重置积分项以避免位置切换时的积分误差
                pos_x_integral = 0;
                pos_y_integral = 0;
                yaw_integral = 0;
                
                Serial.println("到达目标位置，切换到位置保持模式");
            }
        }
        
        // 控制任务执行频率
        wait(1); // 主循环延时10ms
    }
}

bool arrived(void)
{
    /*判断是否到达目标点*/
    // 1. 获取当前点的位置
    global_position_t currentPosition;
    getGlobalPosition(&currentPosition);
    
    // 2. 判断当前点与目标点的距离
    float dx = currentPosition.x - targetPos.x;
    float dy = currentPosition.y - targetPos.y;
    float dyaw = currentPosition.continuousYaw - targetPos.yaw; // 使用rawYaw
    
    // 3. 如果距离小于一定值，则返回true，否则返回false
    const float POSITION_TOLERANCE = 30.0; // 位置容差，单位mm，放宽到30mm
    const float YAW_TOLERANCE = 5.0;     // 偏航角容差，单位为度，放宽到5度
    
    if(abs(dx) < POSITION_TOLERANCE && abs(dy) < POSITION_TOLERANCE && abs(dyaw) < YAW_TOLERANCE) {
        return true;
    } else {
        return false;
    }
}

// 强制停止所有电机的紧急函数，无论当前状态如何
void forceStopAllMotors(void)
{
    if (motor != nullptr) {
        // 立即停止所有电机，多次发送以确保命令被接收
        for (int i = 0; i < 3; i++) {
            motor->stopNow(MOTOR_FR, 0);
            motor->stopNow(MOTOR_FL, 0);
            motor->stopNow(MOTOR_BL, 0);
            motor->stopNow(MOTOR_BR, 0);
            delay(10);
        }
        
        // 禁用运动状态
        isMoving = false;
        isHolding = false;
        
        // 重置所有积分项
        pos_x_integral = 0;
        pos_y_integral = 0;
        yaw_integral = 0;
        
        Serial.println("紧急停止！所有电机已强制停止");
    }
}