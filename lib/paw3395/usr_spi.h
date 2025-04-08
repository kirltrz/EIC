#include "stdint.h"
#include <Arduino.h>
#include <SPI.h>

/*使用的SPI控制器，对于esp32s3 SPI2->FSPI，SPI3->HSPI*/
#define MYSPIBUS HSPI

#define SPI_I2S_FLAG_RXNE ((uint16_t)0x0001)
#define SPI_I2S_FLAG_TXE ((uint16_t)0x0002)
/**********************************************************************************************************/
void SPI_Init(uint8_t sclk, uint8_t miso, uint8_t mosi, uint8_t ncs);
uint8_t SPI_SendReceive(uint8_t data);
uint8_t read_register(uint8_t adress);
void write_register(uint8_t adress, uint8_t vlue);
void cs_high(void);
void cs_low(void);