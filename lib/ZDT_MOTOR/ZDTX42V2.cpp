/**********************************************************
 * ZDT_X42_V2.0步进闭环控制库
 * 编写作者：ZHANGDATOU
 * 技术支持：张大头闭环伺服
 * 淘宝店铺：https://zhangdatou.taobao.com
 * CSDN博客：https://blog.csdn.net/zhangdatou666
 * qq交流群：262438510
 **********************************************************/

#include "ZDTX42V2.h"

// 构造函数，自动初始化
ZDTX42V2::ZDTX42V2(HardwareSerial* serial) {
  // 设置串口
  _serial = serial ? serial : &Serial; // 如果为空则使用默认Serial
  
  // 初始化串口
  _serial->begin(115200);
  
  // 等待串口初始化完成
  while (!(*_serial)) {
    ; // 等待串行端口连接。仅本机USB端口需要
  }
  
  // 上电延时0.5秒等待ZDT_X42_V2闭环初始化完毕
  delay(500);
}

/**
 * @brief    将当前位置清零
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void ZDTX42V2::resetCurPosToZero(uint8_t addr) {
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] = addr;                       // 地址
  cmd[1] = 0x0A;                       // 功能码
  cmd[2] = 0x6D;                       // 辅助码
  cmd[3] = 0x6B;                       // 校验字节
  
  // 发送命令
  _serial->write(cmd, 4);
}

/**
 * @brief    解除堵转保护
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void ZDTX42V2::resetClogPro(uint8_t addr) {
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] = addr;                       // 地址
  cmd[1] = 0x0E;                       // 功能码
  cmd[2] = 0x52;                       // 辅助码
  cmd[3] = 0x6B;                       // 校验字节
  
  // 发送命令
  _serial->write(cmd, 4);
}

/**
 * @brief    读取系统参数
 * @param    addr  ：电机地址
 * @param    s     ：系统参数类型
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void ZDTX42V2::readSysParams(uint8_t addr, SysParams_t s) {
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] = addr;                       // 地址

  switch(s)                             // 功能码
  {
    case S_VER   : cmd[1] = 0x1F; break;                  /* 读取固件版本和对应的硬件版本 */
    case S_RL    : cmd[1] = 0x20; break;                  /* 读取读取相电阻和相电感 */
    case S_PID   : cmd[1] = 0x21; break;                  /* 读取PID参数 */
    case S_ORG   : cmd[1] = 0x22; break;                  /* 读取回零参数 */
    case S_VBUS  : cmd[1] = 0x24; break;                  /* 读取总线电压 */
    case S_CBUS  : cmd[1] = 0x26; break;                  /* 读取总线电流 */
    case S_CPHA  : cmd[1] = 0x27; break;                  /* 读取相电流 */
    case S_ENC   : cmd[1] = 0x29; break;                  /* 读取编码器原始值 */
    case S_CPUL  : cmd[1] = 0x30; break;                  /* 读取实时脉冲数（根据实时位置计算得到的脉冲数） */
    case S_ENCL  : cmd[1] = 0x31; break;                  /* 读取经过线性化校准后的编码器值 */
    case S_TPUL  : cmd[1] = 0x32; break;                  /* 读取输入脉冲数 */
    case S_TPOS  : cmd[1] = 0x33; break;                  /* 读取电机目标位置 */
    case S_OPOS  : cmd[1] = 0x34; break;                  /* 读取电机实时设定的目标位置（开环模式的实时位置） */
    case S_VEL   : cmd[1] = 0x35; break;                  /* 读取电机实时转速 */
    case S_CPOS  : cmd[1] = 0x36; break;                  /* 读取电机实时位置（基于角度编码器累加的电机实时位置） */
    case S_PERR  : cmd[1] = 0x37; break;                  /* 读取电机位置误差 */
    case S_TEMP  : cmd[1] = 0x39; break;                  /* 读取电机实时温度 */
    case S_SFLAG : cmd[1] = 0x3A; break;                  /* 读取状态标志位 */
    case S_OFLAG : cmd[1] = 0x3B; break;                  /* 读取回零状态标志位 */
    case S_Conf  : cmd[1] = 0x42; cmd[2] = 0x6C; break;   /* 读取驱动参数 */
    case S_State : cmd[1] = 0x43; cmd[2] = 0x7A; break;   /* 读取系统状态参数 */
    default: break;
  }

  // 发送命令
  if(s >= S_Conf) {
    cmd[3] = 0x6B; _serial->write(cmd, 4);
  } else {
    cmd[2] = 0x6B; _serial->write(cmd, 3);
  }
}

/**
 * @brief    修改开环/闭环控制模式
 * @param    addr     ：电机地址
 * @param    svF      ：是否存储标志，false为不存储，true为存储
 * @param    ctrl_mode：控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void ZDTX42V2::modifyCtrlMode(uint8_t addr, bool svF, uint8_t ctrl_mode) {
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] = addr;                       // 地址
  cmd[1] = 0x46;                       // 功能码
  cmd[2] = 0x69;                       // 辅助码
  cmd[3] = svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] = ctrl_mode;                  // 控制模式（对应屏幕上的Ctrl_Mode菜单），0是开环模式，1是FOC矢量闭环模式
  cmd[5] = 0x6B;                       // 校验字节
  
  // 发送命令
  _serial->write(cmd, 6);
}

/**
 * @brief    使能信号控制
 * @param    addr  ：电机地址
 * @param    state ：使能状态     ，true为使能电机，false为关闭电机
 * @param    snF   ：多机同步标志 ，0为不启用，其余值启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void ZDTX42V2::enControl(uint8_t addr, bool state, uint8_t snF) {
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] = addr;                       // 地址
  cmd[1] = 0xF3;                       // 功能码
  cmd[2] = 0xAB;                       // 辅助码
  cmd[3] = (uint8_t)state;             // 使能状态
  cmd[4] = snF;                        // 多机同步运动标志
  cmd[5] = 0x6B;                       // 校验字节
  
  // 发送命令
  _serial->write(cmd, 6);
}

/**
 * @brief    力矩模式
 * @param    addr  ：电机地址
 * @param    sign  ：符号         ，0为正，其余值为负
 * @param    t_ramp：斜率(Ma/s)   ，范围0 - 65535Ma/s
 * @param    torque：力矩(Ma)     ，范围0 - 4000Ma
 * @param    snF   ：多机同步标志 ，0为不启用，其余值启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void ZDTX42V2::torqueControl(uint8_t addr, uint8_t sign, uint16_t t_ramp, uint16_t torque, uint8_t snF) {
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] = addr;                       // 地址
  cmd[1] = 0xF5;                       // 功能码
  cmd[2] = sign;                       // 符号（方向）
  cmd[3] = (uint8_t)(t_ramp >> 8);     // 力矩斜率(Ma/s)高8位字节
  cmd[4] = (uint8_t)(t_ramp >> 0);     // 力矩斜率(Ma/s)低8位字节
  cmd[5] = (uint8_t)(torque >> 8);     // 力矩(Ma)高8位字节
  cmd[6] = (uint8_t)(torque >> 0);     // 力矩(Ma)低8位字节
  cmd[7] = snF;                        // 多机同步运动标志
  cmd[8] = 0x6B;                       // 校验字节
  
  // 发送命令
  _serial->write(cmd, 9);
}

/**
 * @brief    速度模式
 * @param    addr  ：电机地址
 * @param    dir  ：方向         ，0为正，其余值为负
 * @param    v_ramp：斜率(RPM/s)   ，范围0 - 65535RPM/s
 * @param    velocity：速度(RPM)     ，范围0 - 65535RPM
 * @param    snF   ：多机同步标志 ，0为不启用，其余值启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void ZDTX42V2::velocityControl(uint8_t addr, uint8_t dir, uint16_t v_ramp, float velocity, uint8_t snF) {
  uint8_t cmd[16] = {0};
  uint16_t vel_int = (uint16_t)velocity;
  
  // 装载命令
  cmd[0] = addr;                       // 地址
  cmd[1] = 0xF6;                       // 功能码
  cmd[2] = dir;                        // 方向
  cmd[3] = (uint8_t)(v_ramp >> 8);     // 速度斜率(RPM/s)高8位字节
  cmd[4] = (uint8_t)(v_ramp >> 0);     // 速度斜率(RPM/s)低8位字节
  cmd[5] = (uint8_t)(vel_int >> 8);    // 速度(RPM)高8位字节
  cmd[6] = (uint8_t)(vel_int >> 0);    // 速度(RPM)低8位字节
  cmd[7] = snF;                        // 多机同步运动标志
  cmd[8] = 0x6B;                       // 校验字节
  
  // 发送命令
  _serial->write(cmd, 9);
}

/**
 * @brief    直通限速位置模式
 * @param    addr    ：电机地址
 * @param    dir     ：方向           ，0为绝对位置模式，1为相对位置正方向，2为相对位置负方向
 * @param    velocity：速度限制(RPM)  ，范围0 - 65535RPM
 * @param    position：位置（°）      ，范围-2,147,483,647 - 2,147,483,647°
 * @param    raf     ：相对绝对标志 ，0为相对位置，1为绝对位置
 * @param    snF     ：多机同步标志 ，0为不启用，其余值启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void ZDTX42V2::bypassPositionLVControl(uint8_t addr, uint8_t dir, float velocity, float position, uint8_t raf, uint8_t snF) {
  uint8_t cmd[16] = {0};
  uint16_t vel_int = (uint16_t)velocity;
  int32_t pos_int = (int32_t)(position * 100);
  
  // 装载命令
  cmd[0] = addr;                       // 地址
  cmd[1] = 0xFD;                       // 功能码
  cmd[2] = dir;                        // 方向
  cmd[3] = (uint8_t)(vel_int >> 8);    // 速度(RPM)高8位字节
  cmd[4] = (uint8_t)(vel_int >> 0);    // 速度(RPM)低8位字节
  cmd[5] = (uint8_t)(pos_int >> 24);   // 位置(°*100)最高8位字节
  cmd[6] = (uint8_t)(pos_int >> 16);   // 位置(°*100)次高8位字节
  cmd[7] = (uint8_t)(pos_int >> 8);    // 位置(°*100)次低8位字节
  cmd[8] = (uint8_t)(pos_int >> 0);    // 位置(°*100)最低8位字节
  cmd[9] = raf;                        // 相对位置标志
  cmd[10] = snF;                       // 多机同步运动标志
  cmd[11] = 0x6B;                      // 校验字节
  
  // 发送命令
  _serial->write(cmd, 12);
}

/**
 * @brief    梯形曲线加减速位置模式
 * @param    addr    ：电机地址
 * @param    dir     ：方向           ，0为绝对位置模式，1为相对位置正方向，2为相对位置负方向
 * @param    acc     ：加速度(RPM/s)  ，范围0 - 65535RPM/s
 * @param    dec     ：减速度(RPM/s)  ，范围0 - 65535RPM/s
 * @param    velocity：速度限制(RPM)  ，范围0 - 65535RPM
 * @param    position：位置（°）      ，范围-2,147,483,647 - 2,147,483,647°
 * @param    raf     ：相对绝对标志 ，0为相对位置，1为绝对位置
 * @param    snF     ：多机同步标志 ，0为不启用，其余值启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void ZDTX42V2::trajPositionControl(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t dec, float velocity, float position, uint8_t raf, uint8_t snF) {
  uint8_t cmd[16] = {0};
  uint16_t vel_int = (uint16_t)velocity;
  int32_t pos_int = (int32_t)(position * 100);
  
  // 装载命令
  cmd[0] = addr;                       // 地址
  cmd[1] = 0xFE;                       // 功能码
  cmd[2] = dir;                        // 方向
  cmd[3] = (uint8_t)(acc >> 8);        // 加速度(RPM/s)高8位字节
  cmd[4] = (uint8_t)(acc >> 0);        // 加速度(RPM/s)低8位字节
  cmd[5] = (uint8_t)(dec >> 8);        // 减速度(RPM/s)高8位字节
  cmd[6] = (uint8_t)(dec >> 0);        // 减速度(RPM/s)低8位字节
  cmd[7] = (uint8_t)(vel_int >> 8);    // 速度(RPM)高8位字节
  cmd[8] = (uint8_t)(vel_int >> 0);    // 速度(RPM)低8位字节
  cmd[9] = (uint8_t)(pos_int >> 24);   // 位置(°*100)最高8位字节
  cmd[10] = (uint8_t)(pos_int >> 16);  // 位置(°*100)次高8位字节
  cmd[11] = (uint8_t)(pos_int >> 8);   // 位置(°*100)次低8位字节
  cmd[12] = (uint8_t)(pos_int >> 0);   // 位置(°*100)最低8位字节
  cmd[13] = raf;                        // 相对位置标志
  cmd[14] = snF;                       // 多机同步运动标志
  cmd[15] = 0x6B;                      // 校验字节
  
  // 发送命令
  _serial->write(cmd, 16);
}

/**
 * @brief    让电机立即停止运动
 * @param    addr  ：电机地址
 * @param    snF   ：多机同步标志，0为不启用，其余值启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void ZDTX42V2::stopNow(uint8_t addr, uint8_t snF) {
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] = addr;                       // 地址
  cmd[1] = 0xF7;                       // 功能码
  cmd[2] = 0x5A;                       // 辅助码
  cmd[3] = snF;                        // 多机同步运动标志
  cmd[4] = 0x6B;                       // 校验字节
  
  // 发送命令
  _serial->write(cmd, 5);
}

/**
 * @brief    触发多机同步开始运动
 * @param    addr  ：电机地址（0为广播地址）
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void ZDTX42V2::synchronousMotion(uint8_t addr) {
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] = addr;                       // 地址
  cmd[1] = 0xFC;                       // 功能码
  cmd[2] = 0x6B;                       // 校验字节
  
  // 发送命令
  _serial->write(cmd, 3);
}

/**
 * @brief    设置单圈回零的零点位置
 * @param    addr  ：电机地址
 * @param    svF   ：是否存储标志，false为不存储，true为存储
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void ZDTX42V2::originSetO(uint8_t addr, bool svF) {
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] = addr;                       // 地址
  cmd[1] = 0xED;                       // 功能码
  cmd[2] = 0x73;                       // 辅助码
  cmd[3] = svF;                        // 是否存储标志
  cmd[4] = 0x6B;                       // 校验字节
  
  // 发送命令
  _serial->write(cmd, 5);
}

/**
 * @brief    修改回零参数
 * @param    addr   ：电机地址
 * @param    svF    ：是否存储标志，false为不存储，true为存储
 * @param    o_mode ：回零模式，0为单圈回零，1为多圈回零，2为限位回零
 * @param    o_dir  ：回零方向，0为顺时针，1为逆时针
 * @param    o_vel  ：回零速度(RPM)，范围0 - 65535RPM
 * @param    o_tm   ：回零超时(ms)，范围0 - 65535*1000ms
 * @param    sl_vel ：寻找限位开关速度(RPM)，范围0 - 65535RPM
 * @param    sl_ma  ：寻找限位开关最大加速度(RPM/s)，范围0 - 65535RPM/s
 * @param    sl_ms  ：寻找限位开关最大减速度(RPM/s)，范围0 - 65535RPM/s
 * @param    potF   ：找零后的位置，false为找零后电机当前位置为0，true为找零后电机当前位置为360*n (绝对位置最近的360倍数，(-180, 180])
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void ZDTX42V2::originModifyParams(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF) {
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] = addr;                       // 地址
  cmd[1] = 0xEB;                       // 功能码
  cmd[2] = 0x64;                       // 辅助码
  cmd[3] = svF;                        // 是否存储标志
  cmd[4] = o_mode;                     // 回零模式
  cmd[5] = o_dir;                      // 回零方向
  cmd[6] = (uint8_t)(o_vel >> 8);      // 回零速度(RPM)高8位字节
  cmd[7] = (uint8_t)(o_vel >> 0);      // 回零速度(RPM)低8位字节
  cmd[8] = (uint8_t)(o_tm >> 16);      // 回零超时(ms)高16位高8位字节
  cmd[9] = (uint8_t)(o_tm >> 8);       // 回零超时(ms)高16位低8位字节
  cmd[10] = (uint8_t)(o_tm >> 0);      // 回零超时(ms)低16位低8位字节
  cmd[11] = (uint8_t)(sl_vel >> 8);    // 寻找限位开关速度(RPM)高8位字节
  cmd[12] = (uint8_t)(sl_vel >> 0);    // 寻找限位开关速度(RPM)低8位字节
  cmd[13] = (uint8_t)(sl_ma >> 8);     // 寻找限位开关最大加速度(RPM/s)高8位字节
  cmd[14] = (uint8_t)(sl_ma >> 0);     // 寻找限位开关最大加速度(RPM/s)低8位字节
  cmd[15] = 0x6B;                      // 校验字节
  
  // 发送命令
  _serial->write(cmd, 16);
}

/**
 * @brief    发送命令触发回零
 * @param    addr   ：电机地址
 * @param    o_mode ：回零模式，0为读取驱动器配置，1为强制单圈回零，2为强制多圈回零，3为强制限位回零
 * @param    snF    ：多机同步标志，0为不启用，其余值启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void ZDTX42V2::originTriggerReturn(uint8_t addr, uint8_t o_mode, bool snF) {
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] = addr;                       // 地址
  cmd[1] = 0xEC;                       // 功能码
  cmd[2] = 0xA9;                       // 辅助码
  cmd[3] = o_mode;                     // 回零模式
  cmd[4] = snF;                        // 多机同步运动标志
  cmd[5] = 0x6B;                       // 校验字节
  
  // 发送命令
  _serial->write(cmd, 6);
}

/**
 * @brief    强制中断并退出回零
 * @param    addr   ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void ZDTX42V2::originInterrupt(uint8_t addr) {
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] = addr;                       // 地址
  cmd[1] = 0xEE;                       // 功能码
  cmd[2] = 0x94;                       // 辅助码
  cmd[3] = 0x6B;                       // 校验字节
  
  // 发送命令
  _serial->write(cmd, 4);
}

/**
 * @brief    接收数据函数
 * @param    rxCmd  ：接收数据缓存数组地址
 * @param    rxCount：接收数据长度地址
 * @retval   无
 */
void ZDTX42V2::receiveData(uint8_t *rxCmd, uint8_t *rxCount) {
  uint32_t startTime = millis();
  
  // 清空接收数据长度
  *rxCount = 0;
  
  // 等待有数据接收
  while(!_serial->available()) {
    // 超时退出
    if(millis() - startTime > 100) return;
  }
  
  // 延时等待所有数据接收完成
  delay(10);
  
  // 接收数据
  while(_serial->available() && *rxCount < 128) {
    rxCmd[*rxCount] = _serial->read();
    (*rxCount)++;
  }
} 