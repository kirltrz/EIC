/**********************************************************
 * ZDT_X42_V2.0步进闭环控制库
 * 编写作者：ZHANGDATOU
 * 技术支持：张大头闭环伺服
 * 淘宝店铺：https://zhangdatou.taobao.com
 * CSDN博客：https://blog.csdn.net/zhangdatou666
 * qq交流群：262438510
 **********************************************************/
#pragma once

#include <Arduino.h>

// 定义绝对值宏
#define ABS(x) ((x) > 0 ? (x) : -(x)) 

// 系统参数枚举
typedef enum {
  S_VER   = 0,      /* 读取固件版本和对应的硬件版本 */
  S_RL    = 1,      /* 读取读取相电阻和相电感 */
  S_PID   = 2,      /* 读取PID参数 */
  S_ORG   = 3,      /* 读取回零参数 */
  S_VBUS  = 4,      /* 读取总线电压 */
  S_CBUS  = 5,      /* 读取总线电流 */
  S_CPHA  = 6,      /* 读取相电流 */
  S_ENC   = 7,      /* 读取编码器原始值 */
  S_CPUL  = 8,      /* 读取实时脉冲数（根据实时位置计算得到的脉冲数） */
  S_ENCL  = 9,      /* 读取经过线性化校准后的编码器值 */
  S_TPUL  = 10,     /* 读取输入脉冲数 */
  S_TPOS  = 11,     /* 读取电机目标位置 */
  S_OPOS  = 12,     /* 读取电机实时设定的目标位置（开环模式的实时位置） */
  S_VEL   = 13,     /* 读取电机实时转速 */
  S_CPOS  = 14,     /* 读取电机实时位置（基于角度编码器累加的电机实时位置） */
  S_PERR  = 15,     /* 读取电机位置误差 */
  S_TEMP  = 16,     /* 读取电机实时温度 */
  S_SFLAG = 17,     /* 读取状态标志位 */
  S_OFLAG = 18,     /* 读取回零状态标志位 */
  S_Conf  = 19,     /* 读取驱动参数 */
  S_State = 20,     /* 读取系统状态参数 */
} SysParams_t;

class ZDTX42V2 {
public:
  // 构造函数自动初始化，可选传入串口指针
  ZDTX42V2(HardwareSerial* serial = nullptr);
  
  // 基本控制函数
  void resetCurPosToZero(uint8_t addr);                          // 将当前位置清零
  void resetClogPro(uint8_t addr);                               // 解除堵转保护
  void readSysParams(uint8_t addr, SysParams_t s);               // 读取参数
  void modifyCtrlMode(uint8_t addr, bool svF, uint8_t ctrl_mode); // 发送命令切换开环/闭环控制模式
  void enControl(uint8_t addr, bool state, uint8_t snF = 0);      // 电机使能控制
  
  // 运动控制函数
  void torqueControl(uint8_t addr, uint8_t sign, uint16_t t_ramp, uint16_t torque, uint8_t snF = 0);                                                       // 力矩模式控制
  void velocityControl(uint8_t addr, uint8_t dir, uint16_t v_ramp, float velocity, uint8_t snF = 0);                                                       // 速度模式控制
  void bypassPositionLVControl(uint8_t addr, uint8_t dir, float velocity, float position, uint8_t raf = 0, uint8_t snF = 0);                              // 直通限速位置模式控制
  void trajPositionControl(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t dec, float velocity, float position, uint8_t raf = 0, uint8_t snF = 0);      // 梯形曲线加减速位置模式控制
  void stopNow(uint8_t addr, uint8_t snF = 0);                                                                                                             // 让电机立即停止运动
  void synchronousMotion(uint8_t addr);                                                                                                                    // 触发多机同步开始运动
  
  // 回零相关函数
  void originSetO(uint8_t addr, bool svF);                                                                                                                                          // 设置单圈回零的零点位置
  void originModifyParams(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);      // 修改回零参数
  void originTriggerReturn(uint8_t addr, uint8_t o_mode, bool snF);                                                                                                                // 发送命令触发回零
  void originInterrupt(uint8_t addr);                                                                                                                                              // 强制中断并退出回零
  
  // 接收数据函数
  void receiveData(uint8_t *rxCmd, uint8_t *rxCount);            // 返回数据接收函数
  uint16_t getVoltage();

private:
  HardwareSerial* _serial;
};
