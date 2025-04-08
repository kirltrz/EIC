#include "usr_delay.h"

void delay_ms(uint16_t nms)
{
    delay(nms);
}

void delay_us(uint32_t nus)
{
    delayMicroseconds(nus);
}

void delay_125_ns(uint8_t nns)
{
    // 优化计算方式，避免浮点运算
    // ESP32 CPU频率为240MHz，每个周期约为4.17ns
    // 125ns约等于30个CPU周期，因此直接计算
    uint32_t cycles = nns * 30; // 125ns * nns / 4.17ns
    uint32_t start = xthal_get_ccount();
    while (xthal_get_ccount() - start < cycles);
}