#include "ZDT_MOTOR_EMM_V5.h"
#include "taskManager.h"
#include "config.h"

// 构造函数，自动初始化
ZDT_MOTOR_EMM_V5::ZDT_MOTOR_EMM_V5(HardwareSerial *serial)
{
  _serial = serial;
  if (_serial == nullptr) {
    _serial = &Serial;
    _serial->begin(MOTOR_BAUDRATE, SERIAL_8N1, PIN_MOTOR_RX, PIN_MOTOR_TX);
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
  // 使用默认重试次数(3次)
  enControlWithRetry(addr, state, snF, 3);
}

/**
 * @brief    带自定义重试次数的使能信号控制
 * @param    addr        ：电机地址
 * @param    state       ：使能状态     ，true为使能电机，false为关闭电机
 * @param    snF         ：多机同步标志 ，0为不启用，其余值启用
 * @param    maxRetries  ：最大重试次数 ，默认为3次
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void ZDT_MOTOR_EMM_V5::enControlWithRetry(uint8_t addr, bool state, uint8_t snF, int maxRetries)
{
  const int RETRY_DELAY = 50;       // 重试间隔时间(ms)
  const int VERIFY_DELAY = 50;      // 验证前等待时间(ms)
  
  // 限制重试次数在合理范围内
  if (maxRetries < 1) maxRetries = 1;
  if (maxRetries > 10) maxRetries = 10;
  
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] = addr;           // 地址
  cmd[1] = 0xF3;           // 功能码
  cmd[2] = 0xAB;           // 辅助码
  cmd[3] = (uint8_t)state; // 使能状态
  cmd[4] = snF;            // 多机同步运动标志
  cmd[5] = 0x6B;           // 校验字节

  // 检查是否为广播地址 - 使用config.h中定义的地址常量
  const uint8_t individual_motors[] = {MOTOR_BL, MOTOR_FL, MOTOR_FR, MOTOR_BR}; // 后左, 前左, 前右, 后右
  const int num_motors = sizeof(individual_motors) / sizeof(individual_motors[0]);

  // 重试逻辑
  for (int attempt = 1; attempt <= maxRetries; attempt++)
  {
    // 发送命令
    sendCommand(cmd, 6);
    
    // 等待电机响应命令
    delay(VERIFY_DELAY);
    
    bool allMotorsSuccess = true;
    int successCount = 0;
    int failureCount = 0;
    
    if (addr == MOTOR_BROADCAST) 
    {
      // 广播地址：逐个验证所有电机
      DEBUG_LOG("广播命令验证开始 (第%d次尝试)", attempt);
      
      for (int i = 0; i < num_motors; i++) 
      {
        uint8_t motorAddr = individual_motors[i];
        bool actualState = isMotorEnabled(motorAddr);
        
        if (actualState == state) 
        {
          successCount++;
          DEBUG_LOG("  电机%d %s验证成功", motorAddr, state ? "使能" : "失能");
        } 
        else 
        {
          failureCount++;
          allMotorsSuccess = false;
          DEBUG_LOG("  电机%d %s验证失败", motorAddr, state ? "使能" : "失能");
        }
      }
      
      DEBUG_LOG("广播命令验证结果: 成功%d个, 失败%d个", successCount, failureCount);
    } 
    else 
    {
      // 单个电机地址：验证指定电机
      bool actualState = isMotorEnabled(addr);
      allMotorsSuccess = (actualState == state);
      
      if (allMotorsSuccess) 
      {
        successCount = 1;
      } 
      else 
      {
        failureCount = 1;
      }
    }
    
    if (allMotorsSuccess) 
    {
      // 状态验证成功
      if (addr == MOTOR_BROADCAST) 
      {
        if (attempt == 1) 
        {
          DEBUG_LOG("广播%s成功，所有%d个电机状态正确", state ? "使能" : "失能", num_motors);
        } 
        else 
        {
          DEBUG_LOG("广播%s成功 (第%d次尝试)，所有%d个电机状态正确", 
                    state ? "使能" : "失能", attempt, num_motors);
        }
      } 
      else 
      {
        if (attempt == 1) 
        {
          DEBUG_LOG("电机%d%s成功", addr, state ? "使能" : "失能");
        } 
        else 
        {
          DEBUG_LOG("电机%d%s成功 (第%d次尝试)", addr, state ? "使能" : "失能", attempt);
        }
      }
      return; // 验证成功，退出函数
    }
    else 
    {
      // 状态验证失败
      if (attempt < maxRetries) 
      {
        // 还有重试机会
        if (addr == MOTOR_BROADCAST) 
        {
          DEBUG_LOG("广播%s验证失败，第%d次尝试，%d个电机失败，准备重试...", 
                    state ? "使能" : "失能", attempt, failureCount);
        } 
        else 
        {
          DEBUG_LOG("电机%d %s验证失败，第%d次尝试，准备重试...", 
                    addr, state ? "使能" : "失能", attempt);
        }
        delay(RETRY_DELAY); // 重试前等待
      } 
      else 
      {
        // 已达到最大重试次数，最终失败
        if (addr == MOTOR_BROADCAST) 
        {
          DEBUG_LOG("错误：广播%s最终失败，已重试%d次，%d个电机失败", 
                    state ? "使能" : "失能", maxRetries, failureCount);
          
          // 显示失败电机的详细状态
          for (int i = 0; i < num_motors; i++) 
          {
            uint8_t motorAddr = individual_motors[i];
            bool actualState = isMotorEnabled(motorAddr);
            
            if (actualState != state) 
            {
              uint8_t fullStatus = getMotorStatus(motorAddr);
              if (fullStatus != 0xFF) 
              {
                DEBUG_LOG("失败电机%d状态详情 - 使能:%s, 到位:%s, 堵转:%s, 堵转保护:%s", 
                          motorAddr,
                          (fullStatus & 0x01) ? "是" : "否",
                          (fullStatus & 0x02) ? "是" : "否", 
                          (fullStatus & 0x04) ? "是" : "否",
                          (fullStatus & 0x08) ? "是" : "否");
              }
              else
              {
                DEBUG_LOG("失败电机%d状态读取失败，可能存在通信问题", motorAddr);
              }
            }
          }
        } 
        else 
        {
          DEBUG_LOG("错误：电机%d%s最终失败，已重试%d次", addr, state ? "使能" : "失能", maxRetries);
          
          // 读取完整状态信息用于调试
          uint8_t fullStatus = getMotorStatus(addr);
          if (fullStatus != 0xFF) 
          {
            DEBUG_LOG("电机%d最终状态详情 - 使能:%s, 到位:%s, 堵转:%s, 堵转保护:%s", 
                      addr,
                      (fullStatus & 0x01) ? "是" : "否",
                      (fullStatus & 0x02) ? "是" : "否", 
                      (fullStatus & 0x04) ? "是" : "否",
                      (fullStatus & 0x08) ? "是" : "否");
          }
          else
          {
            DEBUG_LOG("电机%d状态读取失败，可能存在通信问题", addr);
          }
        }
      }
    }
  }
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

      if ((int)(currentTime - lastTime) > 100) // 100毫秒内串口没有数据进来，就判定一帧数据接收结束
      {
        *rxCount = i; // 数据长度
        return; // 退出循环
      }
    }
  }
}

uint16_t ZDT_MOTOR_EMM_V5::getVoltage(uint8_t addr)
{
  uint16_t voltage = 0;
  uint8_t rxCmd[128] = {0};
  uint8_t rxCount = 0;
  readSysParams(addr, S_VBUS);
  //DEBUG_LOG("发送读取电压命令");
  //wait(100);//貌似不需要额外等待也可以正确接收
  receiveData(rxCmd, &rxCount);
  //DEBUG_LOG("读取电压命令完成");
  // 调试输出接收到的字节
  //DEBUG_LOG("接收到的字节数: %d", rxCount);
  //DEBUG_LOG("字节[1]: 0x%02X", rxCmd[1]);

  if (rxCmd[1] == 0x24)
  {
    //DEBUG_LOG("成功解析电压命令");
    voltage = (rxCmd[2] << 8) | rxCmd[3];
  }
  //DEBUG_LOG("电机%d，电压: %d", addr, voltage);
  return voltage+700/*电机内部电压压降0.7V，在此补偿*/;
}

/**
 * @brief    读取电机状态标志位
 * @param    addr  ：电机地址
 * @retval   电机状态标志字节，0xFF表示读取失败
 */
uint8_t ZDT_MOTOR_EMM_V5::getMotorStatus(uint8_t addr)
{
  const int MAX_READ_RETRIES = 2;   // 状态读取最大重试次数  
  const int READ_RETRY_DELAY = 30;  // 读取重试间隔时间(ms)
  
  uint8_t status = 0xFF; // 默认为失败状态
  
  for (int attempt = 1; attempt <= MAX_READ_RETRIES; attempt++)
  {
    uint8_t rxCmd[128] = {0};
    uint8_t rxCount = 0;
    
    // 发送读取状态标志位命令
    readSysParams(addr, S_FLAG);
    
    // 接收返回数据
    receiveData(rxCmd, &rxCount);
    
    // 检查返回数据格式：地址 + 0x3A + 状态标志 + 校验字节
    if (rxCount >= 4 && rxCmd[0] == addr && rxCmd[1] == 0x3A && rxCmd[3] == 0x6B)
    {
      status = rxCmd[2]; // 状态标志字节
      return status;     // 读取成功，立即返回
    }
    else if (rxCount >= 4 && rxCmd[0] == addr && rxCmd[1] == 0x00 && rxCmd[2] == 0xEE && rxCmd[3] == 0x6B)
    {
      // 错误命令返回：地址 + 0x00 + 0xEE + 校验字节
      DEBUG_LOG("电机%d返回命令错误，可能是不支持的命令", addr);
      return 0xFF; // 表示命令错误，不重试
    }
    else
    {
      // 数据格式不正确或接收失败
      if (attempt < MAX_READ_RETRIES)
      {
        DEBUG_LOG("电机%d状态读取失败(第%d次尝试)，准备重试...", addr, attempt);
        delay(READ_RETRY_DELAY); // 重试前等待
      }
      else
      {
        DEBUG_LOG("电机%d状态读取最终失败，已重试%d次", addr, MAX_READ_RETRIES);
      }
    }
  }
  
  return status; // 返回失败状态
}

/**
 * @brief    判断电机是否使能
 * @param    addr  ：电机地址
 * @retval   true为使能，false为未使能
 */
bool ZDT_MOTOR_EMM_V5::isMotorEnabled(uint8_t addr)
{
  uint8_t status = getMotorStatus(addr);
  if (status == 0xFF) return false; // 读取失败，默认为未使能
  
  // 电机使能状态标志位 = 状态 & 0x01
  return (status & 0x01) != 0;
}

/**
 * @brief    判断电机是否到位
 * @param    addr  ：电机地址
 * @retval   true为到位，false为未到位
 */
bool ZDT_MOTOR_EMM_V5::isMotorInPosition(uint8_t addr)
{
  uint8_t status = getMotorStatus(addr);
  if (status == 0xFF) return false; // 读取失败，默认为未到位
  
  // 电机到位标志位 = 状态 & 0x02
  return (status & 0x02) != 0;
}

/**
 * @brief    判断电机是否堵转
 * @param    addr  ：电机地址
 * @retval   true为堵转，false为未堵转
 */
bool ZDT_MOTOR_EMM_V5::isMotorStalled(uint8_t addr)
{
  uint8_t status = getMotorStatus(addr);
  if (status == 0xFF) return false; // 读取失败，默认为未堵转
  
  // 电机堵转标志位 = 状态 & 0x04
  return (status & 0x04) != 0;
}

/**
 * @brief    判断电机是否堵转保护
 * @param    addr  ：电机地址
 * @retval   true为堵转保护，false为无堵转保护
 */
bool ZDT_MOTOR_EMM_V5::isMotorStallProtected(uint8_t addr)
{
  uint8_t status = getMotorStatus(addr);
  if (status == 0xFF) return false; // 读取失败，默认为无堵转保护
  
  // 电机堵转保护标志 = 状态 & 0x08
  return (status & 0x08) != 0;
}

// 发送命令函数
void ZDT_MOTOR_EMM_V5::sendCommand(uint8_t *cmd, uint8_t len) {
  if (_serial != nullptr) {
    _serial->write(cmd, len);
    delay(2);
    //DEBUG_LOG("发送命令\n");
  }
}
