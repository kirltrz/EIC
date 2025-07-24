#include "ZDT_MOTOR_EMM_V5.h"
#include "taskManager.h"
#include "config.h"

// 全局变量定义
SemaphoreHandle_t motorDataMutex = NULL;
motor_data_cache_t motorDataCache[5] = {0}; // 支持最多4个电机（地址1-4）

// 构造函数，自动初始化
ZDT_MOTOR_EMM_V5::ZDT_MOTOR_EMM_V5(HardwareSerial *serial)
{
  _serial = serial;
  //if (_serial != nullptr) {
  //  _serial->begin(921600);
  //}
  
  // 创建互斥锁（只创建一次）
  if (motorDataMutex == NULL) {
    motorDataMutex = xSemaphoreCreateMutex();
  }
  
  // 初始化缓存数据
  for (int i = 0; i < 5; i++) {
    motorDataCache[i].addr = i;
    motorDataCache[i].velocity = 0;
    motorDataCache[i].voltage = 0;
    motorDataCache[i].lastUpdateTime = 0;
    motorDataCache[i].isValid = false;
  }
}

/**
 * @brief    将当前位置清零
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void ZDT_MOTOR_EMM_V5::resetCurPosToZero(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr; // 地址
  cmd[1] = 0x0A; // 功能码
  cmd[2] = 0x6D; // 辅助码
  cmd[3] = 0x6B; // 校验字节

  // 发送命令
  sendCommand(cmd, 4);
}

/**
 * @brief    解除堵转保护
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void ZDT_MOTOR_EMM_V5::resetClogPro(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr; // 地址
  cmd[1] = 0x0E; // 功能码
  cmd[2] = 0x52; // 辅助码
  cmd[3] = 0x6B; // 校验字节

  // 发送命令
  sendCommand(cmd, 4);
}

/**
 * @brief    读取系统参数
 * @param    addr  ：电机地址
 * @param    s     ：系统参数类型
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void ZDT_MOTOR_EMM_V5::readSysParams(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0;
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[i] = addr; ++i;                   // 地址

  switch(s)                             // 功能码
  {
    case S_VER  : cmd[i] = 0x1F; ++i; break;
    case S_RL   : cmd[i] = 0x20; ++i; break;
    case S_PID  : cmd[i] = 0x21; ++i; break;
    case S_VBUS : cmd[i] = 0x24; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;
    case S_PERR : cmd[i] = 0x37; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_ORG  : cmd[i] = 0x3B; ++i; break;
    case S_Conf : cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break;
    case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
    default: break;
  }

  cmd[i] = 0x6B; ++i;                   // 校验字节
  
  // 发送命令
  sendCommand(cmd, i);
}

/**
 * @brief    修改开环/闭环控制模式
 * @param    addr     ：电机地址
 * @param    svF      ：是否存储标志，false为不存储，true为存储
 * @param    ctrl_mode：控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void ZDT_MOTOR_EMM_V5::modifyCtrlMode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr;      // 地址
  cmd[1] = 0x46;      // 功能码
  cmd[2] = 0x69;      // 辅助码
  cmd[3] = svF;       // 是否存储标志，false为不存储，true为存储
  cmd[4] = ctrl_mode; // 控制模式（对应屏幕上的Ctrl_Mode菜单），0是开环模式，1是FOC矢量闭环模式
  cmd[5] = 0x6B;      // 校验字节

  // 发送命令
  sendCommand(cmd, 6);
}

/**
 * @brief    使能信号控制
 * @param    addr  ：电机地址
 * @param    state ：使能状态     ，true为使能电机，false为关闭电机
 * @param    snF   ：多机同步标志 ，0为不启用，其余值启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void ZDT_MOTOR_EMM_V5::enControl(uint8_t addr, bool state, uint8_t snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr;           // 地址
  cmd[1] = 0xF3;           // 功能码
  cmd[2] = 0xAB;           // 辅助码
  cmd[3] = (uint8_t)state; // 使能状态
  cmd[4] = snF;            // 多机同步运动标志
  cmd[5] = 0x6B;           // 校验字节

  // 发送命令
  sendCommand(cmd, 6);
}

/**
  * @brief    速度模式
  * @param    addr：电机地址
  * @param    dir ：方向       ，0为CW，其余值为CCW
  * @param    vel ：速度       ，范围0 - 5000RPM
  * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动
  * @param    snF ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_MOTOR_EMM_V5::velocityControl(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF6;                       // 功能码
  cmd[2] =  dir;                        // 方向
  cmd[3] =  (uint8_t)(vel >> 8);        // 速度(RPM)高8位字节
  cmd[4] =  (uint8_t)(vel >> 0);        // 速度(RPM)低8位字节
  cmd[5] =  acc;                        // 加速度，注意：0是直接启动
  cmd[6] =  snF;                        // 多机同步运动标志
  cmd[7] =  0x6B;                       // 校验字节
  
  // 发送命令
  sendCommand(cmd, 8);
}

/**
  * @brief    位置模式
  * @param    addr：电机地址
  * @param    dir ：方向        ，0为CW，其余值为CCW
  * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
  * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
  * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
  * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
  * @param    snF ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_MOTOR_EMM_V5::posControl(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFD;                      // 功能码
  cmd[2]  =  dir;                       // 方向
  cmd[3]  =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位字节
  cmd[4]  =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位字节 
  cmd[5]  =  acc;                       // 加速度，注意：0是直接启动
  cmd[6]  =  (uint8_t)(clk >> 24);      // 脉冲数(bit24 - bit31)
  cmd[7]  =  (uint8_t)(clk >> 16);      // 脉冲数(bit16 - bit23)
  cmd[8]  =  (uint8_t)(clk >> 8);       // 脉冲数(bit8  - bit15)
  cmd[9]  =  (uint8_t)(clk >> 0);       // 脉冲数(bit0  - bit7 )
  cmd[10] =  raF;                       // 相位/绝对标志，false为相对运动，true为绝对值运动
  cmd[11] =  snF;                       // 多机同步运动标志，false为不启用，true为启用
  cmd[12] =  0x6B;                      // 校验字节
  
  // 发送命令
  sendCommand(cmd, 13);
}

/**
 * @brief    让电机立即停止运动
 * @param    addr  ：电机地址
 * @param    snF   ：多机同步标志，0为不启用，其余值启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void ZDT_MOTOR_EMM_V5::stopNow(uint8_t addr, uint8_t snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr; // 地址
  cmd[1] = 0xFE; // 功能码
  cmd[2] = 0x98; // 辅助码
  cmd[3] = snF;  // 多机同步运动标志
  cmd[4] = 0x6B; // 校验字节

  // 发送命令
  sendCommand(cmd, 5);
}

/**
 * @brief    触发多机同步开始运动
 * @param    addr  ：电机地址（0为广播地址）
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void ZDT_MOTOR_EMM_V5::synchronousMotion(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr; // 地址
  cmd[1] = 0xFF; // 功能码
  cmd[2] = 0x66; // 辅助码
  cmd[3] = 0x6B; // 校验字节

  // 发送命令
  sendCommand(cmd, 4);
}

/**
 * @brief    设置单圈回零的零点位置
 * @param    addr  ：电机地址
 * @param    svF   ：是否存储标志，false为不存储，true为存储
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void ZDT_MOTOR_EMM_V5::originSetO(uint8_t addr, bool svF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr; // 地址
  cmd[1] = 0x93; // 功能码
  cmd[2] = 0x88; // 辅助码
  cmd[3] = svF;  // 是否存储标志
  cmd[4] = 0x6B; // 校验字节

  // 发送命令
  sendCommand(cmd, 5);
}

/**
  * @brief    修改回零参数
  * @param    addr  ：电机地址
  * @param    svF   ：是否存储标志，false为不存储，true为存储
  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    o_dir  ：回零方向，0为CW，其余值为CCW
  * @param    o_vel  ：回零速度，单位：RPM（转/分钟）
  * @param    o_tm   ：回零超时时间，单位：毫秒
  * @param    sl_vel ：无限位碰撞回零检测转速，单位：RPM（转/分钟）
  * @param    sl_ma  ：无限位碰撞回零检测电流，单位：Ma（毫安）
  * @param    sl_ms  ：无限位碰撞回零检测时间，单位：Ms（毫秒）
  * @param    potF   ：上电自动触发回零，false为不使能，true为使能
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_MOTOR_EMM_V5::originModifyParams(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  uint8_t cmd[32] = {0};

  // 装载命令
  cmd[0] = addr;                    // 地址
  cmd[1] = 0x4C;                    // 功能码
  cmd[2] = 0xAE;                    // 辅助码
  cmd[3] = svF;                     // 是否存储标志，false为不存储，true为存储
  cmd[4] = o_mode;                  // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  cmd[5] = o_dir;                   // 回零方向
  cmd[6] = (uint8_t)(o_vel >> 8);   // 回零速度(RPM)高8位字节
  cmd[7] = (uint8_t)(o_vel >> 0);   // 回零速度(RPM)低8位字节
  cmd[8] = (uint8_t)(o_tm >> 24);   // 回零超时时间(bit24 - bit31)
  cmd[9] = (uint8_t)(o_tm >> 16);   // 回零超时时间(bit16 - bit23)
  cmd[10] = (uint8_t)(o_tm >> 8);   // 回零超时时间(bit8  - bit15)
  cmd[11] = (uint8_t)(o_tm >> 0);   // 回零超时时间(bit0  - bit7 )
  cmd[12] = (uint8_t)(sl_vel >> 8); // 无限位碰撞回零检测转速(RPM)高8位字节
  cmd[13] = (uint8_t)(sl_vel >> 0); // 无限位碰撞回零检测转速(RPM)低8位字节
  cmd[14] = (uint8_t)(sl_ma >> 8);  // 无限位碰撞回零检测电流(Ma)高8位字节
  cmd[15] = (uint8_t)(sl_ma >> 0);  // 无限位碰撞回零检测电流(Ma)低8位字节
  cmd[16] = (uint8_t)(sl_ms >> 8);  // 无限位碰撞回零检测时间(Ms)高8位字节
  cmd[17] = (uint8_t)(sl_ms >> 0);  // 无限位碰撞回零检测时间(Ms)低8位字节
  cmd[18] = potF;                   // 上电自动触发回零，false为不使能，true为使能
  cmd[19] = 0x6B;                   // 校验字节

  // 发送命令
  sendCommand(cmd, 20);
}

/**
 * @brief    发送命令触发回零
 * @param    addr   ：电机地址
 * @param    o_mode ：回零模式，0为读取驱动器配置，1为强制单圈回零，2为强制多圈回零，3为强制限位回零
 * @param    snF    ：多机同步标志，0为不启用，其余值启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void ZDT_MOTOR_EMM_V5::originTriggerReturn(uint8_t addr, uint8_t o_mode, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr;   // 地址
  cmd[1] = 0x9A;   // 功能码
  cmd[2] = o_mode; // 回零模式
  cmd[3] = snF;    // 多机同步运动标志
  cmd[4] = 0x6B;   // 校验字节

  // 发送命令
  sendCommand(cmd, 5);
}

/**
 * @brief    强制中断并退出回零
 * @param    addr   ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void ZDT_MOTOR_EMM_V5::originInterrupt(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr; // 地址
  cmd[1] = 0x9C; // 功能码
  cmd[2] = 0x48; // 辅助码
  cmd[3] = 0x6B; // 校验字节

  // 发送命令
  sendCommand(cmd, 4);
}

/**
 * @brief    接收数据函数
 * @param    rxCmd  ：接收数据缓存数组地址
 * @param    rxCount：接收数据长度地址
 * @retval   无
 */
void ZDT_MOTOR_EMM_V5::receiveData(uint8_t *rxCmd, uint8_t *rxCount)
{
  unsigned long lastTime;    // 上一时刻的时间
  unsigned long currentTime; // 当前时刻的时间

  // 记录当前的时间
  currentTime = millis();
  lastTime = currentTime;
  
  // 开始接收数据
  for (int i = 0; i < 128; i++)
  {
    if (_serial->available() > 0) // 串口有数据进来
    {
      rxCmd[i] = _serial->read(); // 接收数据
      if (rxCmd[i] == 0x6B)       // 如果校验字节为0x6B，则一帧数据接收结束
      {
        *rxCount = i + 1;
        return;
      }
      lastTime = millis(); // 更新上一时刻的时间
    }
    else // 串口有没有数据
    {
      currentTime = millis(); // 获取当前时刻的时间

      if ((int)(currentTime - lastTime) > 20) // 20毫秒内串口没有数据进来，就判定一帧数据接收结束
      {
        *rxCount = i; // 数据长度
        return; // 退出循环
      }
    }
  }
}

uint16_t ZDT_MOTOR_EMM_V5::getVoltage(uint8_t addr)
{
  // 验证地址范围
  if (addr < 1 || addr > 4) {
    return 0;
  }
  
  // 发送查询指令（非阻塞）
  readSysParams(addr, S_VBUS);
  
  // 等待一小段时间让监听任务有机会接收数据
  vTaskDelay(pdMS_TO_TICKS(5));
  
  // 从缓存中读取电压数据
  uint16_t voltage = 0;
  if (xSemaphoreTake(motorDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    // 检查数据是否有效且未过期（1秒内的数据认为有效）
    if (motorDataCache[addr].isValid && 
        (millis() - motorDataCache[addr].lastUpdateTime) < 1000) {
      voltage = motorDataCache[addr].voltage;
    }
    xSemaphoreGive(motorDataMutex);
  }
  
  return voltage;
}
/**
 * @return 转速，单位RPM
 */
int ZDT_MOTOR_EMM_V5::getVelocity(uint8_t addr)
{
  // 验证地址范围
  if (addr < 1 || addr > 4) {
    return 0;
  }
  
  // 发送查询指令（非阻塞）
  readSysParams(addr, S_VEL);
  
  // 等待一小段时间让监听任务有机会接收数据
  vTaskDelay(pdMS_TO_TICKS(5));
  
  // 从缓存中读取速度数据
  int velocity = 0;
  if (xSemaphoreTake(motorDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    // 检查数据是否有效且未过期（100ms内的数据认为有效）
    if (motorDataCache[addr].isValid && 
        (millis() - motorDataCache[addr].lastUpdateTime) < 100) {
      velocity = motorDataCache[addr].velocity;
    }
    xSemaphoreGive(motorDataMutex);
  }
  
  return velocity;
}
// 发送命令函数
void ZDT_MOTOR_EMM_V5::sendCommand(uint8_t *cmd, uint8_t len) {
  if (_serial != nullptr) {
    _serial->write(cmd, len);
    delay(3);//不延迟的话指令会乱
    //DEBUG_LOG("发送命令\n");
  }
}

/**
 * @brief 非阻塞式数据接收，持续监听串口并解析数据包
 */
void ZDT_MOTOR_EMM_V5::receiveDataNonBlocking(void)
{
  static uint8_t rxBuffer[128];
  static uint8_t rxIndex = 0;
  static unsigned long lastByteTime = 0;
  
  // 检查是否有数据可读
  if (_serial->available() == 0) {
    // 检查超时，如果超过20ms没收到数据且缓冲区有数据，则重置
    if (rxIndex > 0 && (millis() - lastByteTime) > 20) {
      rxIndex = 0;
    }
    return;
  }
  
  // 读取一个字节
  uint8_t byte = _serial->read();
  lastByteTime = millis();
  
  // 存储字节
  rxBuffer[rxIndex] = byte;
  rxIndex++;
  
  // 检查是否收到校验字节0x6B（一帧数据结束）
  if (byte == 0x6B && rxIndex >= 4) {
    // 解析数据包
    uint8_t addr = rxBuffer[0];
    uint8_t cmd = rxBuffer[1];
    
    // 验证地址范围
    if (addr >= 1 && addr <= 4) {
      bool dataUpdated = false;
      
      if (cmd == 0x35 && rxIndex >= 6) { // 速度数据
        uint16_t velocity = (rxBuffer[3] << 8) | rxBuffer[4];
        int ultimateVelocity = rxBuffer[2] == 0x00 ? velocity : -velocity;
        
        // 更新缓存
        if (xSemaphoreTake(motorDataMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
          motorDataCache[addr].velocity = ultimateVelocity;
          motorDataCache[addr].lastUpdateTime = millis();
          motorDataCache[addr].isValid = true;
          xSemaphoreGive(motorDataMutex);
          dataUpdated = true;
        }
      }
      else if (cmd == 0x24 && rxIndex >= 5) { // 电压数据
        uint16_t voltage = (rxBuffer[2] << 8) | rxBuffer[3];
        
        // 更新缓存
        if (xSemaphoreTake(motorDataMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
          motorDataCache[addr].voltage = voltage + 700; // 补偿电压压降
          motorDataCache[addr].lastUpdateTime = millis();
          motorDataCache[addr].isValid = true;
          xSemaphoreGive(motorDataMutex);
          dataUpdated = true;
        }
      }
    }
    
    // 重置缓冲区
    rxIndex = 0;
  }
  else if (rxIndex >= 128) {
    // 缓冲区溢出，重置
    rxIndex = 0;
  }
}

/**
 * @brief 电机数据监听任务 - 纯粹的数据接收监听，不主动发送指令
 */
void ZDT_MOTOR_EMM_V5::motorDataListenerTask(void *pvParameters)
{
  ZDT_MOTOR_EMM_V5* motorInstance = (ZDT_MOTOR_EMM_V5*)pvParameters;
  
  // 等待电机初始化完成
  while (motorInstance->_serial == nullptr) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  
  // 清空串口缓冲区
  while (motorInstance->_serial->available() > 0) {
    motorInstance->_serial->read();
  }
  
  DEBUG_LOG("电机数据监听任务已启动");
  
  while (true) {
    // 纯粹的非阻塞接收数据，不主动发送任何指令
    motorInstance->receiveDataNonBlocking();
    
    // 短暂延迟，让出CPU时间
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
