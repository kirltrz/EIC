#include "motion.h"
#include "sensor.h"
#include "taskManager.h" // 导入wait函数
#include <esp_timer.h>

// 声明ZDT_MOTOR控制对象指针
ZDT_MOTOR_EMM_V5 *motor = nullptr;

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

// 定义每个电机的地址
static const uint8_t motor_addrs[4] = {MOTOR_FR, MOTOR_BR, MOTOR_BL, MOTOR_FL};

static global_position_t currentPosition; // 实时全局位置
volatile float wheel_speeds[4] = {0};     // 电机实时速度

// 内部PID参数
float _posKp, _posKi, _posKd, _yawKp, _yawKi, _yawKd;
float _max_linear_speed, _max_angular_speed;

const int PID_UPDATE_RATE = 100;                 // PID更新频率，100Hz
const int PID_INTERVAL = 1000 / PID_UPDATE_RATE; // PID更新间隔，10ms

// unsigned long currentTime = 0;
unsigned long lastPID_Time = 0; // 上一次计算及控制时间

// PID相关变量
float pos_x_error, pos_y_error, yaw_error;                // 误差
float pos_x_derivative, pos_y_derivative, yaw_derivative; // 微分项
float global_vx, global_vy, omega;                        // 全局速度(场地坐标系)
float linear_speed, scale;                                // 线性速度
float yaw_rad, local_vx, local_vy;                        // 局部速度（小车坐标系）
float speed;                                              // 速度
uint8_t dir;                                              // 每个电机运动的方向

// 控制任务执行周期
TickType_t xLastWakeTime;
const TickType_t xFrequency = pdMS_TO_TICKS(PID_INTERVAL);

void initMotor()
{
    // 初始化串口通信，用于与电机通信
    MOTOR_SERIAL.begin(SERIAL_BAUDRATE, SERIAL_8N1, PIN_MOTOR_RX, PIN_MOTOR_TX);
    // delay(1000); // 等待通信稳定

    DEBUG_LOG("开始初始化电机...");

    // 初始化电机对象
    motor = new ZDT_MOTOR_EMM_V5(&MOTOR_SERIAL);

    delay(500); // 等待电机初始化完成

    // 确保所有电机都停止
    motor->stopNow(MOTOR_BROADCAST, 0);

    DEBUG_LOG("电机初始化和通信测试完成");
}

// 将全局坐标系中的速度转换为机器人局部坐标系中的速度
void globalToLocalVelocity(float global_vx, float global_vy, float yaw_rad, float &local_vx, float &local_vy)
{
    // yaw_rad是机器人相对于全局坐标系的偏航角，单位是弧度
    local_vx = global_vx * cos(yaw_rad) + global_vy * sin(yaw_rad);
    local_vy = -global_vx * sin(yaw_rad) + global_vy * cos(yaw_rad);
}

// 限制输入值在最小和最大值之间
float constrain_float(float value, float min_value, float max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

void moveTo(POS pos)
{
    /*移动到目标点，仅用于传入目标点*/
    // 保存目标位置
    targetPos = pos;

    // 设置运动状态
    isMoving = true;
    isHolding = false;

    // 重置PID参数
    resetPIDparam();
}

void resetPIDparam()
{
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
    if (motor != nullptr)
    {
        // 停止所有电机 - 使用停止命令
        motor->stopNow(MOTOR_BROADCAST, 0);
        delay(20); // 等待停止命令执行

        // 输出调试信息
        DEBUG_LOG("电机已停止");
    }

    isMoving = false;
    isHolding = false;
}

/*移动任务,不断向电机发送控制指令*/
void moveTask(void *pvParameters)
{
    // 确保电机控制对象已初始化
    unsigned long startTime = millis();
    while (motor == nullptr)
    {
        delay(10);
        if (millis() - startTime > 10000) // 10秒超时
        {
            DEBUG_LOG("电机初始化超时，移动任务可能出错\n");
            break;
        }
    }

    // 任务启动时强制停止所有电机并重置状态
    stopMotion();

    xLastWakeTime = xTaskGetTickCount(); // 初始化xLastWakeTime变量
    while (1)
    {
        if (isMoving || isHolding)
        {
            // 获取当前位置
            getGlobalPosition(&currentPosition);
            if (isMoving)
            {
                // 运动模式PID参数
                _posKp = POS_KP;
                _posKi = POS_KI;
                _posKd = POS_KD;
                _yawKp = YAW_KP;
                _yawKi = YAW_KI;
                _yawKd = YAW_KD;
                _max_linear_speed = MAX_LINEAR_SPEED;
                _max_angular_speed = MAX_ANGULAR_SPEED;
            }
            else if (isHolding)
            {
                // 位置保持模式PID参数
                _posKp = HOLD_POS_KP;
                _posKi = HOLD_POS_KI;
                _posKd = HOLD_POS_KD;
                _yawKp = HOLD_YAW_KP;
                _yawKi = HOLD_YAW_KI;
                _yawKd = HOLD_YAW_KD;
                _max_linear_speed = HOLD_MAX_LINEAR_SPEED;
                _max_angular_speed = HOLD_MAX_ANGULAR_SPEED;
            }
            // 计算误差
            pos_x_error = targetPos.x - currentPosition.x;
            pos_y_error = targetPos.y - currentPosition.y;
            yaw_error = targetPos.yaw - currentPosition.continuousYaw;

            // 更新积分项
            pos_x_integral += pos_x_error * PID_INTERVAL / 1000.0f;
            pos_y_integral += pos_y_error * PID_INTERVAL / 1000.0f;
            yaw_integral += yaw_error * PID_INTERVAL / 1000.0f;

            // 限制积分项防止积分饱和，根据最大速度限制
            pos_x_integral = constrain_float(pos_x_integral, -(_max_linear_speed * 0.3 / _posKi), (_max_linear_speed * 0.3 / _posKi));
            pos_y_integral = constrain_float(pos_y_integral, -(_max_linear_speed * 0.3 / _posKi), (_max_linear_speed * 0.3 / _posKi));
            yaw_integral = constrain_float(yaw_integral, -(_max_angular_speed * 0.3 / _yawKi), (_max_angular_speed * 0.3 / _yawKi));

            // 计算微分项
            // pos_x_derivative = (pos_x_error - pos_x_last_error) * 1000.0f / PID_INTERVAL;
            float alpha = 0.2f; // 滤波系数
            float raw_x_derivative = (pos_x_error - pos_x_last_error) * 1000.0f / PID_INTERVAL;
            pos_x_derivative = alpha * raw_x_derivative + (1 - alpha) * pos_x_derivative;
            // pos_y_derivative = (pos_y_error - pos_y_last_error) * 1000.0f / PID_INTERVAL;
            float raw_y_derivative = (pos_y_error - pos_y_last_error) * 1000.0f / PID_INTERVAL;
            pos_y_derivative = alpha * raw_y_derivative + (1 - alpha) * pos_y_derivative;
            // yaw_derivative = (yaw_error - yaw_last_error) * 1000.0f / PID_INTERVAL;
            float raw_yaw_derivative = (yaw_error - yaw_last_error) * 1000.0f / PID_INTERVAL;
            yaw_derivative=alpha * raw_yaw_derivative + (1 - alpha) * yaw_derivative;

            // 保存当前误差
            pos_x_last_error = pos_x_error;
            pos_y_last_error = pos_y_error;
            yaw_last_error = yaw_error;

            // 计算PID输出 - 这是全局坐标系中的速度
            global_vx = _posKp * pos_x_error + _posKi * pos_x_integral + _posKd * pos_x_derivative;
            global_vy = _posKp * pos_y_error + _posKi * pos_y_integral + _posKd * pos_y_derivative;
            omega = _yawKp * yaw_error + _yawKi * yaw_integral + _yawKd * yaw_derivative;

            // 限制最大线速度
            linear_speed = sqrt(global_vx * global_vx + global_vy * global_vy);
            if (linear_speed > _max_linear_speed)
            {
                scale = _max_linear_speed / linear_speed;
                global_vx *= scale;
                global_vy *= scale;
            }

            // 限制最大角速度
            omega = constrain_float(omega, -_max_angular_speed, _max_angular_speed);

            // 将角度从度转换为弧度
            yaw_rad = currentPosition.rawYaw * DEG_TO_RAD;

            // 计算局部坐标系速度
            globalToLocalVelocity(global_vx, global_vy, yaw_rad, local_vx, local_vy);

            // 计算轮子角速度 (rad/s)
            wheel_speeds[0] = (-local_vy - omega * PI / 180.0f * ROBOT_RADIUS) / WHEEL_RADIUS; // 后左轮 (BL) - 1号
            wheel_speeds[1] = (-local_vx - omega * PI / 180.0f * ROBOT_RADIUS) / WHEEL_RADIUS; // 前左轮 (FL) - 2号
            wheel_speeds[2] = (local_vy - omega * PI / 180.0f * ROBOT_RADIUS) / WHEEL_RADIUS;  // 前右轮 (FR) - 3号
            wheel_speeds[3] = (local_vx - omega * PI / 180.0f * ROBOT_RADIUS) / WHEEL_RADIUS;  // 后右轮 (BR) - 4号

            // 直接使用速度控制模式控制电机
            for (int i = 0; i < 4; i++)
            {
                speed = wheel_speeds[i] * 60.0f / (2.0f * PI); // RAD/s转换为RPM

                // 限制最小速度
                if (abs(speed) < MIN_SPEED_RPM && abs(speed) > 0.01f)
                {
                    speed = (speed > 0) ? MIN_SPEED_RPM : -MIN_SPEED_RPM;
                }

                // 如果速度几乎为零，不发送命令
                if (abs(speed) < 0.01f)
                {
                    continue;
                }

                // 限制最大速度
                speed = constrain_float(speed, -MAX_SPEED_RPM, MAX_SPEED_RPM);

                // 确定方向和速度值
                dir = (speed >= 0) ? 0 : 1; // 不知道怎么转的，根据实际情况修改

                // 使用速度控制模式
                motor->velocityControl(motor_addrs[i], dir, abs(speed), DEFAULT_ACC, 0);

                /* 接收会导致重启
                // 在receiveData之前添加
                memset(rxCmd, 0, sizeof(rxCmd)); // 每次使用前清空

                // 接收并处理来自电机的反馈数据
                motor->receiveData(rxCmd, &rxCount);

                // 限制rxCount大小
                if (rxCount >= sizeof(rxCmd))
                    rxCount = sizeof(rxCmd) - 1;
                */

                // 添加调试信息，显示电机回复
                /*if (i == 0 && (currentTime % 1000) < 10) { // 每秒输出一次第一个电机的回复情况
                    DEBUG_LOG("电机回复数据: 长度=");
                    DEBUG_LOG(rxCount);
                    DEBUG_LOG(", 数据:");
                    for(uint8_t j = 0; j < rxCount && j < 10; j++) {
                        DEBUG_LOG(" 0x");
                        DEBUG_LOG(rxCmd[j], HEX);
                    }
                    Serial.println();
                }*/
            }

            // 判断是否到达目标点
            /*if (isMoving && arrived())
            {
                // 切换到位置保持模式，而不是停止电机
                isMoving = false;
                isHolding = true;

                // 重置积分项以避免位置切换时的积分误差
                pos_x_integral = 0;
                pos_y_integral = 0;
                yaw_integral = 0;

                DEBUG_SERIAL.println("到达目标位置，切换到位置保持模式");
            }*/
        }

        // 使用vTaskDelayUntil严格控制执行周期为10ms
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

bool arrived(void)
{
    /*判断是否到达目标点*/
    // 1. 获取当前点的位置
    // 使用全局变量currentPosition

    // 2. 判断当前点与目标点的距离
    float dx = currentPosition.x - targetPos.x;
    float dy = currentPosition.y - targetPos.y;
    float dyaw = currentPosition.continuousYaw - targetPos.yaw;

    // 3. 如果距离小于一定值，则返回true，否则返回false
    const float POSITION_TOLERANCE = 30.0; // 位置容差，单位mm，放宽到30mm
    const float YAW_TOLERANCE = 5.0;       // 偏航角容差，单位为度，放宽到5度
    const uint32_t ARRIVED_TIME = 500;     // 到达判定时间，单位ms

    static uint32_t arrived_time = 0;         // 记录首次满足条件的时间
    static bool is_arrived_condition = false; // 记录是否曾经满足条件

    bool current_condition = (abs(dx) < POSITION_TOLERANCE && abs(dy) < POSITION_TOLERANCE && abs(dyaw) < YAW_TOLERANCE);

    if (current_condition)
    {
        // 满足条件
        if (!is_arrived_condition)
        {
            // 首次满足条件，记录时间
            arrived_time = millis();
            is_arrived_condition = true;
        }
        else
        {
            // 已经满足过条件，检查是否过了500ms
            if (millis() - arrived_time >= ARRIVED_TIME)
            {
                return true;
            }
        }
        return false;
    }
    else
    {
        // 不满足条件，重置状态
        is_arrived_condition = false;
        return false;
    }
}

// 强制停止所有电机的紧急函数，无论当前状态如何
void forceStopAllMotors(void)
{
    if (motor != nullptr)
    {
        motor->enControl(MOTOR_BROADCAST, false); // 失能驱动板，禁用所有电机

        // 禁用运动状态
        isMoving = false;
        isHolding = false;

        // 重置所有积分项
        pos_x_integral = 0;
        pos_y_integral = 0;
        yaw_integral = 0;

        DEBUG_SERIAL.println("紧急停止！所有电机已强制停止");
    }
}