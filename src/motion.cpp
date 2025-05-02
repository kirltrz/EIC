#include "motion.h"
#include "sensor.h"
#include "config.h"
#include "math.h"
#include "taskManager.h"
/*有待完善 部分函数可以加入pid控制 部分参数待调整 速度模式和位置模式也还未限幅 主循环中也可以添加安全检查*/
ZDTX42V2 *motor = nullptr;
SemaphoreHandle_t targetPositionMutex = NULL; // 目标坐标互斥锁

systemState currentState = IDLE; // 默认空闲

// 数据缓冲区
uint8_t rxBuffer[128];
uint8_t rxLength;
// 定义电机地址数组
const uint8_t motorAddresses[] = {MOTOR_FL, MOTOR_FR, MOTOR_BR, MOTOR_BL};
const int motorCount = sizeof(motorAddresses) / sizeof(motorAddresses[0]);
static POS targetPos = {0, 0, 0};
// 电机当前角度位置(度)
float motorPositions[4] = {0.0f, 0.0f, 0.0f, 0.0f};

void initMotor(void)
{
  // 初始化串口通信，用于与电机通信
  Serial2.begin(SERIAL_BAUDRATE, SERIAL_8N1, PIN_MOTOR_RX /*RX*/, PIN_MOTOR_TX /*TX*/);
  delay(100); // 等待通信稳定

  DEBUG_LOG("开始初始化电机...");
  // 初始化电机对象
  motor = new ZDTX42V2(&Serial2);
  targetPositionMutex = xSemaphoreCreateMutex(); // 创建目标坐标互斥锁
  delay(1000); // 等待电机初始化完成

  // 确保所有电机都停止
  motor->stopNow(MOTOR_BROADCAST, 0);
  //motor->receiveData(rxBuffer, &rxLength);
  // 重置电机位置
  motor->resetCurPosToZero(MOTOR_BROADCAST);
  //motor->receiveData(rxBuffer, &rxLength);

  // delay(500); // 等待电机初始化完成
  DEBUG_LOG("电机初始化和通信测试完成");
}

void moveTo(POS _targetPos)
{ // 不要直接赋值，加互斥锁
  if (xSemaphoreTake(targetPositionMutex, portMAX_DELAY) == pdTRUE)
  {
    targetPos = _targetPos;
    xSemaphoreGive(targetPositionMutex);
  }
}
void stopMotion(void)
{
  global_position_t currentPosition;
  getGlobalPosition(&currentPosition);
  if (xSemaphoreTake(targetPositionMutex, portMAX_DELAY) == pdTRUE)
  {
    targetPos.x = currentPosition.x;
    targetPos.y = currentPosition.y;
    targetPos.yaw = currentPosition.rawYaw;
    xSemaphoreGive(targetPositionMutex);
  }
}
void emergencyStopMotor(bool is_enable)
{ // 失能驱动板
  DEBUG_LOG("急停");
  motor->enControl(MOTOR_BROADCAST, is_enable);
}
// 将全局坐标系中的速度转换为机器人局部坐标系中的速度
void globalToLocalVelocity(float global_vx, float global_vy, float yaw_rad, float &local_vx, float &local_vy)
{
  // yaw_rad是机器人相对于全局坐标系的偏航角，单位是弧度
  // 全局坐标到局部坐标的旋转变换，交换X和Y的映射关系
  local_vx = -global_vy * cos(yaw_rad) + global_vx * sin(yaw_rad);
  local_vy = -global_vx * cos(yaw_rad) - global_vy * sin(yaw_rad);
}
void calculateWheelVelocities(float vx, float vy, float omega, float wheelVelocities[4])
{

  float omegaRad = omega * M_PI / 180.0f; // 将角速度转换为弧度/s
  global_position_t currentPosition;
  getGlobalPosition(&currentPosition);
  float local_vx, local_vy;
  globalToLocalVelocity(vx, vy, currentPosition.rawYaw, local_vx, local_vy);

  // 计算轮子角速度 (rad/s)
  float wheel_speeds[4];

  // 根据轮子布局进行正确的运动学逆解算
  // 1号轮在左下角(0,0)，轮面与y轴平行 - 后左轮 (BL)
  // 2号轮在左上角(0,L)，轮面与x轴平行 - 前左轮 (FL)
  // 3号轮在右上角(L,L)，轮面与y轴平行 - 前右轮 (FR)
  // 4号轮在右下角(L,0)，轮面与x轴平行 - 后右轮 (BR)

  // 翻转X轴控制方向，修正左右行驶方向
  // 交换X和Y对轮子的影响关系
  wheel_speeds[0] = (local_vx - omegaRad * ROBOT_RADIUS) / WHEEL_RADIUS;  // 后左轮 (BL) - 1号
  wheel_speeds[1] = (local_vy - omegaRad * ROBOT_RADIUS) / WHEEL_RADIUS;  // 前左轮 (FL) - 2号
  wheel_speeds[2] = (-local_vx - omegaRad * ROBOT_RADIUS) / WHEEL_RADIUS; // 前右轮 (FR) - 3号
  wheel_speeds[3] = (-local_vy - omegaRad * ROBOT_RADIUS) / WHEEL_RADIUS; // 后右轮 (BR) - 4号

  // 将角速度转换为RPM
  for (int i = 0; i < 4; i++)
  {
    wheelVelocities[i] = wheel_speeds[i] * 60.0f / (2.0f * PI); // 转换为RPM
  }
}

// 计算电机的目标位置(电机的旋转度数)
void calculateWheelPositions(float wheelPositions[4])
{
  // 获取当前全局坐标系下的位置
  global_position_t currentPosition;
  getGlobalPosition(&currentPosition);

  // 将目标位置从全局坐标系转换到机器人局部坐标系
  float localX = (targetPos.x - currentPosition.x) * cos(currentPosition.rawYaw) +
                 (targetPos.y - currentPosition.y) * sin(currentPosition.rawYaw);
  float localY = -(targetPos.x - currentPosition.x) * sin(currentPosition.rawYaw) +
                 (targetPos.y - currentPosition.y) * cos(currentPosition.rawYaw);
  float localTheta = (targetPos.yaw - currentPosition.rawYaw);

  // 计算各轮子位移(mm)
  // 此处位移模型解算不一定准确 虚空开发T—T
  // 轮子1：左前(+,+)  轮子2：右前(+,-)  轮子3：右后(-,-)  轮子4：左后(-,+)
  float wheel1 = localX + localY + (WHEEL_BASE_X + WHEEL_BASE_Y) * localTheta;
  float wheel2 = localX - localY + (WHEEL_BASE_X + WHEEL_BASE_Y) * localTheta;
  float wheel3 = localX - localY - (WHEEL_BASE_X + WHEEL_BASE_Y) * localTheta;
  float wheel4 = localX + localY - (WHEEL_BASE_X + WHEEL_BASE_Y) * localTheta;

  // 转换为电机旋转角度(度)
  wheelPositions[0] = wheel1 / (WHEEL_RADIUS * M_PI / 180.0f * GEAR_RATIO) + motorPositions[0];
  wheelPositions[1] = wheel2 / (WHEEL_RADIUS * M_PI / 180.0f * GEAR_RATIO) + motorPositions[1];
  wheelPositions[2] = wheel3 / (WHEEL_RADIUS * M_PI / 180.0f * GEAR_RATIO) + motorPositions[2];
  wheelPositions[3] = wheel4 / (WHEEL_RADIUS * M_PI / 180.0f * GEAR_RATIO) + motorPositions[3];
}
// 计算两点间距离
float calculateDistance(float x1, float y1, float x2, float y2)
{
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
// 执行粗定位，使用位置模式(梯形加减速)
bool coarsePositioning(void)
{
  // 获取当前全局坐标系下的位置
  global_position_t currentPosition;
  getGlobalPosition(&currentPosition);

  // 计算当前位置和目标位置之间的距离
  float distance = calculateDistance(currentPosition.x, currentPosition.y, targetPos.x, targetPos.y);
  float angleDistance = fabs(currentPosition.rawYaw - targetPos.yaw);

  // 检查是否已经达到粗定位精度
  if (distance <= POSITION_TOLERANCE && angleDistance <= 5.0f)
  {
    return true; // 粗定位完成
  }

  float wheelPositions[4]; // 张大头驱动中的位置模式中的位置参数为电机的旋转度，因此需要将目标点的位置信息转换为四个电机的旋转度
  calculateWheelPositions(wheelPositions);

  for (int i = 0; i < motorCount; i++)
  {
    motor->trajPositionControl(motorAddresses[i], 0, ACC_VALUE, DEC_VALUE, 240, wheelPositions[i], 1, 1);
    //motor->receiveData(rxBuffer, &rxLength);
  }
  motor->synchronousMotion(MOTOR_BROADCAST);
  //motor->receiveData(rxBuffer, &rxLength);

  return false; // 粗定位未完成
}
// 执行精定位，使用速度模式
bool finePositioning(void)
{
  // 获取当前全局坐标系下的位置
  global_position_t currentPosition;
  getGlobalPosition(&currentPosition);

  // 计算当前位置和目标位置之间的距离
  float distance = calculateDistance(currentPosition.x, currentPosition.y, targetPos.x, targetPos.y);
  float angleDistance = fabs(currentPosition.rawYaw - targetPos.yaw);

  // 检查是否已经达到精定位精度
  if (distance <= FINE_POSITION_TOLERANCE && angleDistance <= 1.0f)
  {
    // 达到精度要求，停止所有电机
    // 使用广播地址停止所有电机
    motor->velocityControl(MOTOR_BROADCAST, 0, FINE_VELOCITY_RAMP, 0.0f, 0);
    //motor->receiveData(rxBuffer, &rxLength);

    return true; // 精定位完成
  }
  // 计算根据误差的速度缩放因子(越接近目标速度越慢)(具体参数待调整，可以考虑用增量式pid控制)
  float speedFactor = min(1.0f, max(distance / 50.0f, 0.1f));

  // 计算指向目标的速度向量
  float dx = targetPos.x - currentPosition.x;
  float dy = targetPos.y - currentPosition.y;
  float dtheta = targetPos.yaw - currentPosition.rawYaw;

  // 归一化位移向量
  float length = sqrt(dx * dx + dy * dy);
  if (length > 0)
  {
    dx /= length;
    dy /= length;
  }

  // 计算轮子速度
  float wheelVelocities[4];

  // 计算线速度和角速度分量(mm/s和deg/s)
  float linearSpeed = min(FINE_VELOCITY * speedFactor, FINE_VELOCITY);
  float angularSpeed = min(fabs(dtheta) * 5.0f, 30.0f) * (dtheta >= 0 ? 1 : -1);

  // 转换为轮速,并且将全局坐标系下的速度转换为局部坐标系下的速度
  calculateWheelVelocities(dx * linearSpeed, dy * linearSpeed, angularSpeed, wheelVelocities);

  // 应用最小速度阈值，确保电机不会因速度太小而无法移动
  for (int i = 0; i < 4; i++)
  {
    if (fabs(wheelVelocities[i]) < MIN_VELOCITY && wheelVelocities[i] != 0)
    {
      wheelVelocities[i] = wheelVelocities[i] > 0 ? MIN_VELOCITY : -MIN_VELOCITY;
    }
  }

  for (int i = 0; i < motorCount; i++)
  {
    motor->velocityControl(motorAddresses[i], wheelVelocities[i] < 0, FINE_VELOCITY_RAMP, fabs(wheelVelocities[i]), 0);
    //motor->receiveData(rxBuffer, &rxLength);
  }

  return false; // 精定位未完成
}
/*
void updateCurrentPosition()
{
  // 获取当前全局坐标系下的位置
  global_position_t currentPosition;
  getGlobalPosition(&currentPosition);
  // 计算指向目标的速度向量
  float dx = targetPos.x - currentPosition.x;
  float dy = targetPos.y - currentPosition.y;
  float dtheta = targetPos.yaw - currentPosition.rawYaw;

  if (currentState == COARSE_POSITIONING)
  {
    // 粗定位阶段位置更新较快
    currentPosition.x += dx * 0.05;
    currentPosition.y += dy * 0.05;
    currentPosition.rawYaw += dtheta * 0.05;
  }
  else if (currentState == FINE_POSITIONING)
  {
    // 精定位阶段位置更新较慢
    currentPosition.x += dx * 0.02;
    currentPosition.y += dy * 0.02;
    currentPosition.rawYaw += dtheta * 0.02;
  }
}*/

void moveTask(void *pvParameters)
{
  static unsigned long lastControlTime = 0;
  static unsigned long lastFeedbackTime = 0;
  while (1)
  {
    unsigned long currentTime = millis();

    // 控制周期
    if (currentTime - lastControlTime >= CONTROL_INTERVAL)
    {
      lastControlTime = currentTime;

      // 状态机控制
      switch (currentState)
      {
      case IDLE:
        //DEBUG_LOG("开始粗定位阶段...");
        //currentState = COARSE_POSITIONING;
        break;

      case COARSE_POSITIONING:
        // 执行粗定位
        if (coarsePositioning())
        {
          DEBUG_LOG("粗定位完成，开始精定位阶段...");
          currentState = FINE_POSITIONING;
        }
        break;

      case FINE_POSITIONING:
        // 执行精定位
        if (finePositioning())
        {
          DEBUG_LOG("精定位完成，任务完成!");
          //currentState = COMPLETED;
        }
        break;

      case COMPLETED:
        motor->stopNow(MOTOR_BROADCAST, 0);
        //wait(10);
        break;
      }
    } /*移动任务,不断向电机发送控制指令*/
    // 定期更新和显示当前位置
    if (currentTime - lastFeedbackTime >= FEEDBACK_INTERVAL)
    {
      lastFeedbackTime = currentTime;
      //updateCurrentPosition();
    }
    wait(10);
  }
}
