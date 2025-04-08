# ZDTX42V2 Arduino库

ZDTX42V2是一个用于控制ZDT_X42_V2.0步进闭环电机的Arduino库。该库封装了ZDT_X42_V2.0步进闭环控制器的串口通信协议，使用户能够轻松地通过Arduino控制电机。

## 功能特点

- 支持多种控制模式：位置控制、速度控制、力矩控制
- 支持电机参数读取：电压、电流、位置、速度等
- 支持回零操作和回零参数设置
- 支持多机同步控制
- 兼容TTL、RS232、RS485通信方式

## 安装

1. 下载本库的zip文件
2. 在Arduino IDE中，选择 "项目" -> "加载库" -> "添加.ZIP库..."
3. 选择下载的zip文件
4. 重启Arduino IDE

## 接线说明

Arduino接线：
- Arduino的1引脚接闭环的R/A/H
- Arduino的0引脚接闭环的T/B/L
- Arduino的Gnd接闭环Gnd（共地）

注意：如果是RS485接线，可以不用共地，因为RS485是差分传输。

## 使用示例

### 速度控制

```cpp
#include "ZDTX42V2.h"

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  ZDT.begin();  // 初始化，默认波特率115200
}

void loop() {
  uint8_t rxCmd[128] = {0}; 
  uint8_t rxCount = 0;
  
  // 设置电机以3000RPM的速度旋转
  ZDT.velocityControl(1, 0, 1000, 3000.0f);
  ZDT.receiveData(rxCmd, &rxCount);
  
  delay(3000);
  
  // 停止电机
  ZDT.velocityControl(1, 0, 1000, 0.0f);
  ZDT.receiveData(rxCmd, &rxCount);
  
  delay(3000);
}
```

### 位置控制

```cpp
#include "ZDTX42V2.h"

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  ZDT.begin();
}

void loop() {
  uint8_t rxCmd[128] = {0}; 
  uint8_t rxCount = 0;
  
  // 电机顺时针旋转360度
  ZDT.trajPositionControl(1, 1, 2000, 2000, 3000.0f, 360.0f, 0);
  ZDT.receiveData(rxCmd, &rxCount);
  
  delay(3000);
  
  // 电机逆时针旋转360度
  ZDT.trajPositionControl(1, 2, 2000, 2000, 3000.0f, 360.0f, 0);
  ZDT.receiveData(rxCmd, &rxCount);
  
  delay(3000);
}
```

## API参考

### 基础函数

- `void begin(unsigned long baudRate = 115200)` - 初始化库，设置波特率
- `void receiveData(uint8_t *rxCmd, uint8_t *rxCount)` - 接收数据函数

### 基本控制

- `void resetCurPosToZero(uint8_t addr)` - 将当前位置清零
- `void resetClogPro(uint8_t addr)` - 解除堵转保护
- `void readSysParams(uint8_t addr, SysParams_t s)` - 读取系统参数
- `void modifyCtrlMode(uint8_t addr, bool svF, uint8_t ctrl_mode)` - 修改控制模式
- `void enControl(uint8_t addr, bool state, uint8_t snF = 0)` - 使能信号控制

### 运动控制

- `void torqueControl(uint8_t addr, uint8_t sign, uint16_t t_ramp, uint16_t torque, uint8_t snF = 0)` - 力矩模式控制
- `void velocityControl(uint8_t addr, uint8_t dir, uint16_t v_ramp, float velocity, uint8_t snF = 0)` - 速度模式控制
- `void bypassPositionLVControl(uint8_t addr, uint8_t dir, float velocity, float position, uint8_t raf = 0, uint8_t snF = 0)` - 直通限速位置模式控制
- `void trajPositionControl(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t dec, float velocity, float position, uint8_t raf = 0, uint8_t snF = 0)` - 梯形曲线加减速位置模式控制
- `void stopNow(uint8_t addr, uint8_t snF = 0)` - 让电机立即停止运动
- `void synchronousMotion(uint8_t addr)` - 触发多机同步开始运动

### 回零控制

- `void originSetO(uint8_t addr, bool svF)` - 设置单圈回零的零点位置
- `void originModifyParams(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)` - 修改回零参数
- `void originTriggerReturn(uint8_t addr, uint8_t o_mode, bool snF)` - 发送命令触发回零
- `void originInterrupt(uint8_t addr)` - 强制中断并退出回零

## 常见问题

1. **上传代码时出现错误**：串口和USB下载口共用串口（0,1），上传程序时，先拔掉串口线。
2. **通信不成功**：检查接线是否正确，波特率是否设置为115200。
3. **电机不响应**：检查地址是否正确，默认地址为1。

## 关于作者

- 编写作者：ZHANGDATOU
- 技术支持：张大头闭环伺服
- 淘宝店铺：https://zhangdatou.taobao.com
- CSDN博客：https://blog.csdn.net/zhangdatou666
- QQ交流群：262438510 