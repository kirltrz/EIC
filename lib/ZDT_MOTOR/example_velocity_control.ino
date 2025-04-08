/**********************************************************
 * ZDT_X42_V2.0步进闭环控制库 - 速度控制示例
 * 编写作者：ZHANGDATOU
 * 技术支持：张大头闭环伺服
 * 淘宝店铺：https://zhangdatou.taobao.com
 * CSDN博客：https://blog.csdn.net/zhangdatou666
 * qq交流群：262438510
 **********************************************************/

#include "ZDTX42V2.h"

void setup() {
  // 初始化LED灯
  pinMode(LED_BUILTIN, OUTPUT);
  
  // 初始化ZDT库，设置波特率为115200（默认值）
  ZDT.begin();
}

void loop() {
  // 定义接收数据数组、接收数据长度
  uint8_t rxCmd[128] = {0}; 
  uint8_t rxCount = 0;
  
  // 速度模式：速度斜率1000RPM/s，速度3000RPM
  ZDT.velocityControl(1, 0, 1000, 3000.0f);

  // 等待返回命令，命令数据缓存在数组rxCmd上，长度为rxCount
  ZDT.receiveData(rxCmd, &rxCount);

  // 验证校验字节，验证成功则点亮LED灯，否则熄灭LED灯
  if(rxCmd[rxCount - 1] == 0x6B) { 
    digitalWrite(LED_BUILTIN, HIGH); 
  } else { 
    digitalWrite(LED_BUILTIN, LOW); 
  }

  // 延时3秒
  delay(3000);
  
  // 停止电机
  ZDT.velocityControl(1, 0, 1000, 0.0f);
  
  // 等待返回命令
  ZDT.receiveData(rxCmd, &rxCount);
  
  // 延时3秒
  delay(3000);
} 