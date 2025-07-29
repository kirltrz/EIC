#pragma once
#include "stdint.h"
#include "usr_delay.h"
// 寄存器地址
#define PAW3395_SPIREGISTER_MOTION 0x02
#define PAW3395_SPIREGISTER_MotionBurst 0x16
#define PAW3395_SPIREGISTER_POWERUPRESET 0x3A
/*******************CPI配置寄存器********************/
#define SET_RESOLUTION 0x47
#define RESOLUTION_X_LOW 0x48
#define RESOLUTION_X_HIGH 0x49
#define RESOLUTION_Y_LOW 0x4A
#define RESOLUTION_Y_HIGH 0x4B
#define RIPPLE_CONTROL 0x5A
#define MOTION_CTRL 0x5C
/*******************抬起截止高度寄存器********************/
#define LIFT_CUTOFF_CONFIG 0x4E    // 抬起截止高度配置寄存器
/*******************手动校准相关寄存器********************/
#define SQUAL_REGISTER 0x07        // Surface Quality寄存器
#define CALIBRATION_STATUS_REG 0x4C // 校准状态寄存器（Bank 0x04）
#define CALIBRATION_RESULT_REG 0x4D // 校准结果寄存器（Bank 0x04）
// 寄存器值
#define PAW3395_POWERUPRESET_POWERON 0x5A
/*******************抬起截止高度相关值********************/
#define LIFT_CUTOFF_1MM 0x00       // 1mm抬起截止高度 (低2位: 00)
#define LIFT_CUTOFF_2MM 0x02       // 2mm抬起截止高度 (低2位: 10)
#define LIFT_CUTOFF_MASK 0x03      // 低2位掩码
/*******************手动校准相关值********************/
#define CALIBRATION_SUCCESS 0x05   // 校准成功状态值
#define UNIVERSAL_1MM_SETTING 0x08 // 通用1mm设置值
#define MIN_MOTION_SAMPLES 3000    // 固件辅助校准所需的最小样本数
// 寄存器位
#define PAW3395_OP_MODE0 0
#define PAW3395_OP_MODE1 1
#define PAW3395_PG_FIRST 6
#define PAW3395_PG_VALID 7
// 操作延时
/// SPI读地址延迟
#define PAW3395_TIMINGS_SRAD 2 // 2μs
/// 读取和后续命令之间的SPI延时
#define PAW3395_TIMINGS_SRWSRR 2 // 2μs
/// 写入命令之间的SPI延时
#define PAW3395_TIMINGS_SWW 5 // 5μs
/// 写入和读取命令之间的SPI延时
#define PAW3395_TIMINGS_SWR 5 // 5μs
/// SPI，NCS到SCLK活动/非活动
#define PAW3395_TIMINGS_NCS_SCLK 1 // 120ns
/// 为设置NCS为高电平而退出运动突发模式的延时
#define PAW3395_TIMINGS_BEXIT 4 // 500ns
#define PAW3395_SPI_WRITE 0x80

// Motion Burst数据结构
typedef struct {
    uint8_t motion;
    uint8_t observation;
    int16_t delta_x;
    int16_t delta_y;
    uint8_t squal;
    uint8_t rawdata_sum;
    uint8_t maximum_rawdata;
    uint8_t minimum_rawdata;
    uint8_t shutter_upper;
    uint8_t shutter_lower;
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t squal2;
} motion_burst_data_t;

void paw3395Init(uint16_t dpi, uint8_t nrst, uint8_t ncs, uint8_t sclk, uint8_t miso, uint8_t mosi);                                        // 初始化传感器
void Motion_Burst(int16_t *dx, int16_t *dy); // 读取dx和dy

void Power_up_sequence(void);           // 上电
bool paw3395_check(void);              // 检查paw3395是否正常工作
void Pixel_Burst_Read(uint8_t *pFrame); // 输出原始数据
void DPI_Config(uint16_t CPI_Num);      // 设置DPI

// 抬起截止高度设置功能
void Set_Lift_Cutoff_Height(uint8_t height_mm);  // 设置抬起截止高度 (1mm或2mm)
uint8_t Get_Lift_Cutoff_Height(void);            // 获取当前抬起截止高度设置 (返回1或2，表示mm)

// 手动抬起截止校准功能
bool Manual_Lift_Cutoff_Calibration(void);       // 执行手动抬起截止校准
void Motion_Burst_Extended(motion_burst_data_t *data); // 扩展的Motion Burst读取（包含SQUAL等）
bool Firmware_Aided_Calibration(uint8_t *varA, uint8_t *varB, uint8_t *varC); // 固件辅助校准（可选）
void Enable_Lift_Cutoff_Calibration(uint8_t varA, uint8_t varB, uint8_t varC); // 启用校准设置
void Disable_Lift_Cutoff_Calibration(void);      // 禁用校准设置，恢复通用1mm设置