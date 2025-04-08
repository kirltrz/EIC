#include "usr_spi.h"
#include "usr_delay.h"

// 静态变量，只在当前文件内可见
static uint8_t ncs_pin;

SPISettings paw3395Settings(
    10000000, // 10MHz时钟 (APB时钟80MHz / 8)
    MSBFIRST, // MSB优先
    SPI_MODE3 // CPOL=1, CPHA=1
);

SPIClass mySPI(MYSPIBUS);

// CS 控制函数实现
void cs_high(void) {
    digitalWrite(ncs_pin, HIGH);
}

void cs_low(void) {
    digitalWrite(ncs_pin, LOW);
}

void SPI_Init(uint8_t sclk, uint8_t miso, uint8_t mosi, uint8_t ncs)
{
    // 保存 CS 引脚号
    ncs_pin = ncs;
    
    // 配置 CS 引脚为输出模式
    pinMode(ncs, OUTPUT);
    
    mySPI.begin(sclk, miso, mosi);         // 初始化SPI
    mySPI.setDataMode(SPI_MODE3);          // SPI模式3
    mySPI.setClockDivider(SPI_CLOCK_DIV8); // 设置SPI时钟分频
}

// SPI发送和接收数据
uint8_t SPI_SendReceive(uint8_t data)
{
    return mySPI.transfer(data);
}

// 读取寄存器
uint8_t read_register(uint8_t address)
{
    uint8_t temp;
    mySPI.beginTransaction(paw3395Settings);
    cs_low();

    // 根据数据手册要求的最小延时（125ns）
    delay_125_ns(1);

    mySPI.transfer(address); // 发送读地址（bit7=0）
    delayMicroseconds(5);    // 保持延时满足t_SRAD

    temp = mySPI.transfer(0xFF); // 获取数据
    cs_high();
    mySPI.endTransaction();

    return temp;
}

// 写入寄存器
void write_register(uint8_t address, uint8_t value)
{
    mySPI.beginTransaction(paw3395Settings);
    cs_low();

    // 根据数据手册要求的最小延时（125ns）
    delay_125_ns(1); // 近似替代125ns延时

    mySPI.transfer(address | 0x80); // 写地址（bit7=1）
    mySPI.transfer(value);

    cs_high();
    mySPI.endTransaction();

    delayMicroseconds(5); // 满足t_SWW时序要求
}

void SPI_SendData(uint8_t data)
{
    mySPI.transfer(data);
}

uint8_t SPI_ReceiveData()
{
    return mySPI.transfer(0xFF); // 发送0xFF以接收数据
}