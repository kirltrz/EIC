/**********************************************************
 * ZDT_X42_V2.0步进闭环控制库 - 位置控制示例
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
  
  // 梯形曲线位置模式：加速度2000RPM/s，减速度2000RPM/s，速度3000RPM，相对位置360度
  // 参数说明：地址1，方向1(相对位置正方向)，加速度2000，减速度2000，速度3000，位置360度，相对位置标志0
  ZDT.trajPositionControl(1, 1, 2000, 2000, 3000.0f, 360.0f, 0);

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
  
  // 梯形曲线位置模式：加速度2000RPM/s，减速度2000RPM/s，速度3000RPM，相对位置-360度
  // 参数说明：地址1，方向2(相对位置负方向)，加速度2000，减速度2000，速度3000，位置360度，相对位置标志0
  ZDT.trajPositionControl(1, 2, 2000, 2000, 3000.0f, 360.0f, 0);
  
  // 等待返回命令
  ZDT.receiveData(rxCmd, &rxCount);
  
  // 延时3秒
  delay(3000);
} 