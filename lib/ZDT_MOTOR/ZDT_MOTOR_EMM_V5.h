/**********************************************************
 * ZDT_MOTOR_EMM_V5步进闭环控制库
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
  S_VBUS  = 3,      /* 读取总线电压 */
  S_CPHA  = 5,      /* 读取相电流 */
  S_ENCL  = 7,      /* 读取经过线性化校准后的编码器值 */
  S_TPOS  = 8,      /* 读取电机目标位置角度 */
  S_VEL   = 9,      /* 读取电机实时转速 */
  S_CPOS  = 10,     /* 读取电机实时位置角度 */
  S_PERR  = 11,     /* 读取电机位置误差角度 */
  S_FLAG  = 13,     /* 读取使能/到位/堵转状态标志位 */
  S_Conf  = 14,     /* 读取驱动参数 */
  S_State = 15,     /* 读取系统状态参数 */
  S_ORG   = 16,     /* 读取正在回零/回零失败状态标志位 */
}SysParams_t;

class ZDT_MOTOR_EMM_V5 {
public:
  // 构造函数自动初始化，可选传入串口指针
  ZDT_MOTOR_EMM_V5(HardwareSerial* serial = nullptr);
  
  // 发送命令
  void sendCommand(uint8_t *cmd, uint8_t len);
  
  // 基本控制函数
  void resetCurPosToZero(uint8_t addr);                          // 将当前位置清零
  void resetClogPro(uint8_t addr);                               // 解除堵转保护
  void readSysParams(uint8_t addr, SysParams_t s);               // 读取参数
  void modifyCtrlMode(uint8_t addr, bool svF, uint8_t ctrl_mode); // 发送命令切换开环/闭环控制模式
  void enControl(uint8_t addr, bool state, uint8_t snF = 0);      // 电机使能控制
  
  // 运动控制函数
  void velocityControl(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF = false);                          // 速度模式控制
  void posControl(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);              // 位置模式控制
  void stopNow(uint8_t addr, uint8_t snF = 0);                                                                           // 让电机立即停止运动
  void synchronousMotion(uint8_t addr);                                                                                  // 触发多机同步开始运动
  
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
