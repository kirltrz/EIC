#include "PAW3395.h"
#include "usr_spi.h"

static void Power_Up_Initializaton_Register_Setting(void);

void paw3395Init(uint16_t dpi, uint8_t nrst, uint8_t ncs, uint8_t sclk, uint8_t miso, uint8_t mosi)
{
	// 初始化SPI接口
	SPI_Init(sclk, miso, mosi, ncs);

	// 配置引脚模式
	pinMode(nrst, OUTPUT);

	// 复位传感器
	digitalWrite(nrst, LOW);
	delay(5);
	digitalWrite(nrst, HIGH);

	// 确保片选信号为高电平
	cs_high();

	// 执行上电序列
	Power_up_sequence();

	// 配置DPI设置
	DPI_Config(dpi);
}

/*
 * 严格按照开机顺序执行：
 * 虽然芯片执行内部上电自复位，但仍建议将Power_Up_Reset
 * 每次上电时都会写入寄存器。推荐的芯片上电顺序如下:
 * 1. 以任何顺序为VDD和VDDIO供电，每次供电之间的延迟不超过100ms。确保所有供应稳定。
 * 2. 等待至少50毫秒。
 * 3. 将NCS拉高，然后拉低以重置SPI端口。
 * 4. 将0x5A写入Power_Up_Reset寄存器（或者切换NRESET引脚）。
 * 5. 等待至少5ms。
 * 6. 加载上电初始化寄存器设置。
 * 7. 无论运动位状态如何，都读取寄存器0x02、0x03、0x04、0x05和0x06一次。
 */
void Power_up_sequence(void)
{
	// 步骤2：等待至少50毫秒
	delay_ms(50);

	// 步骤3：重置SPI端口
	cs_high();
	delay_125_ns(PAW3395_TIMINGS_NCS_SCLK);
	cs_low();
	delay_125_ns(PAW3395_TIMINGS_NCS_SCLK);

	// 步骤4：将0x5A写入Power_Up_Reset寄存器
	write_register(PAW3395_SPIREGISTER_POWERUPRESET, PAW3395_POWERUPRESET_POWERON);

	// 步骤5：等待至少5ms
	delay_ms(5);

	// 步骤6：加载上电初始化寄存器设置
	Power_Up_Initializaton_Register_Setting();
	cs_high();
	delay_125_ns(PAW3395_TIMINGS_NCS_SCLK);

	// 步骤7：读取寄存器0x02至0x06
	for (uint8_t reg_it = 0x02; reg_it <= 0x06; reg_it++)
	{
		read_register(reg_it);
		delay_us(PAW3395_TIMINGS_SRWSRR);
	}

	// 片选拉高，结束SPI通讯
	cs_high();
	delay_125_ns(PAW3395_TIMINGS_BEXIT);
}

bool paw3395_check(void)
{
	/*通过读取产品id判断是否正常工作，根据数据手册，产品id寄存器为0x00，产品id为0x51*/
	uint8_t product_id = read_register(0x00);
	if (product_id != 0x51)
	{
		return false;
	}
	return true;
}

/*
 * 严格执行启动Motion Burst的程序：
 * 1. 降低NCS
 * 2. 等待tNCS-SCLK
 * 3. 发送Motion_Burst地址(0x16)，发送后MOSI保持静态直到传输完成
 * 4. 等待tSRAD
 * 5. 连续读取最多12个字节的SPI数据，可通过将NCS拉高至少tBEXIT来终止
 * 6. 读取新数据时从步骤1重复
 *
 * 注意：无论在运行或静止模式下，都可以从Burst_Motion_Read寄存器读取数据
 */
void Motion_Burst(volatile int16_t *dx, volatile int16_t *dy)
{
	uint8_t buffer[12] = {0};

	// 片选拉低，开始SPI通信
	cs_low();
	delay_125_ns(PAW3395_TIMINGS_NCS_SCLK);

	// 发送运动突发读取命令
	SPI_SendReceive(PAW3395_SPIREGISTER_MotionBurst);
	delay_us(PAW3395_TIMINGS_SRAD);

	// 连续读取12字节数据
	for (uint8_t i = 0; i < 12; i++)
	{
		buffer[i] = SPI_SendReceive(0x00);
	}

	// 片选拉高，结束SPI通信
	cs_high();
	delay_125_ns(PAW3395_TIMINGS_BEXIT);

	// 解析X和Y方向的位移数据
	*dx = -(int16_t)(buffer[2] | (buffer[3] << 8));
	*dy = (int16_t)(buffer[4] | (buffer[5] << 8));
}

/*
 *上电初始化寄存器设置
 */
static void Power_Up_Initializaton_Register_Setting(void)
{
	uint8_t read_tmp;
	uint8_t i;

	// 初始化寄存器配置
	const uint8_t reg_config[][2] = {
		{0x7F, 0x07}, {0x40, 0x41}, {0x7F, 0x00}, {0x40, 0x80}, {0x7F, 0x0E}, {0x55, 0x0D}, {0x56, 0x1B}, {0x57, 0xE8}, {0x58, 0xD5}, {0x7F, 0x14}, {0x42, 0xBC}, {0x43, 0x74}, {0x4B, 0x20}, {0x4D, 0x00}, {0x53, 0x0E}, {0x7F, 0x05}, {0x44, 0x04}, {0x4D, 0x06}, {0x51, 0x40}, {0x53, 0x40}, {0x55, 0xCA}, {0x5A, 0xE8}, {0x5B, 0xEA}, {0x61, 0x31}, {0x62, 0x64}, {0x6D, 0xB8}, {0x6E, 0x0F}, {0x70, 0x02}, {0x4A, 0x2A}, {0x60, 0x26}, {0x7F, 0x06}, {0x6D, 0x70}, {0x6E, 0x60}, {0x6F, 0x04}, {0x53, 0x02}, {0x55, 0x11}, {0x7A, 0x01}, {0x7D, 0x51}, {0x7F, 0x07}, {0x41, 0x10}, {0x42, 0x32}, {0x43, 0x00}, {0x7F, 0x08}, {0x71, 0x4F}, {0x7F, 0x09}, {0x62, 0x1F}, {0x63, 0x1F}, {0x65, 0x03}, {0x66, 0x03}, {0x67, 0x1F}, {0x68, 0x1F}, {0x69, 0x03}, {0x6A, 0x03}, {0x6C, 0x1F}, {0x6D, 0x1F}, {0x51, 0x04}, {0x53, 0x20}, {0x54, 0x20}, {0x71, 0x0C}, {0x72, 0x07}, {0x73, 0x07}, {0x7F, 0x0A}, {0x4A, 0x14}, {0x4C, 0x14}, {0x55, 0x19}, {0x7F, 0x14}, {0x4B, 0x30}, {0x4C, 0x03}, {0x61, 0x0B}, {0x62, 0x0A}, {0x63, 0x02}, {0x7F, 0x15}, {0x4C, 0x02}, {0x56, 0x02}, {0x41, 0x91}, {0x4D, 0x0A}, {0x7F, 0x0C}, {0x4A, 0x10}, {0x4B, 0x0C}, {0x4C, 0x40}, {0x41, 0x25}, {0x55, 0x18}, {0x56, 0x14}, {0x49, 0x0A}, {0x42, 0x00}, {0x43, 0x2D}, {0x44, 0x0C}, {0x54, 0x1A}, {0x5A, 0x0D}, {0x5F, 0x1E}, {0x5B, 0x05}, {0x5E, 0x0F}, {0x7F, 0x0D}, {0x48, 0xDD}, {0x4F, 0x03}, {0x52, 0x49}, {0x51, 0x00}, {0x54, 0x5B}, {0x53, 0x00}, {0x56, 0x64}, {0x55, 0x00}, {0x58, 0xA5}, {0x57, 0x02}, {0x5A, 0x29}, {0x5B, 0x47}, {0x5C, 0x81}, {0x5D, 0x40}, {0x71, 0xDC}, {0x70, 0x07}, {0x73, 0x00}, {0x72, 0x08}, {0x75, 0xDC}, {0x74, 0x07}, {0x77, 0x00}, {0x76, 0x08}, {0x7F, 0x10}, {0x4C, 0xD0}, {0x7F, 0x00}, {0x4F, 0x63}, {0x4E, 0x00}, {0x52, 0x63}, {0x51, 0x00}, {0x54, 0x54}, {0x5A, 0x10}, {0x77, 0x4F}, {0x47, 0x01}, {0x5B, 0x40}, {0x64, 0x60}, {0x65, 0x06}, {0x66, 0x13}, {0x67, 0x0F}, {0x78, 0x01}, {0x79, 0x9C}, {0x40, 0x00}, {0x55, 0x02}, {0x23, 0x70}, {0x22, 0x01}};

	// 写入所有配置寄存器
	for (i = 0; i < sizeof(reg_config) / sizeof(reg_config[0]); i++)
	{
		write_register(reg_config[i][0], reg_config[i][1]);
	}

	// 等待1ms
	delay_ms(1);

	// 等待寄存器0x6C值变为0x80，最多等待60ms
	for (i = 0; i < 60; i++)
	{
		read_tmp = read_register(0x6C);
		if (read_tmp == 0x80)
			break;
		delay_ms(1);
	}

	// 超时处理
	if (i == 60)
	{
		write_register(0x7F, 0x14);
		write_register(0x6C, 0x00);
		write_register(0x7F, 0x00);
	}

	// 最终配置
	write_register(0x22, 0x00);
	write_register(0x55, 0x00);
	write_register(0x7F, 0x07);
	write_register(0x40, 0x40);
	write_register(0x7F, 0x00);
}

/*
 *RawData输出程序
 *	读取当前鼠标摄像头拍摄到的接触面的像素阵列信息
 *	像素信息存入数组pFrame中
 *
 *注意：
 *	在RawData输出过程中，必须将鼠标置于静止位置
 */
void Pixel_Burst_Read(uint8_t *pFrame)
{
	uint8_t reg_tmp;

	// 拉低片选信号
	cs_low();
	// 等待片选到时钟的建立时间
	delay_125_ns(PAW3395_TIMINGS_NCS_SCLK);

	// 切换到Bank 0并设置操作模式
	write_register(0x7F, 0x00);
	write_register(0x40, 0x80);

	// 等待传感器进入正确的操作模式
	do
	{
		reg_tmp = read_register(PAW3395_SPIREGISTER_MOTION);
		delay_us(PAW3395_TIMINGS_SRWSRR);
	} while ((reg_tmp & ((1 << PAW3395_OP_MODE0) | (1 << PAW3395_OP_MODE1))) != 0);

	// 配置像素读取参数
	write_register(0x50, 0x01); // 启用像素读取
	write_register(0x55, 0x04); // 设置读取模式
	write_register(0x58, 0xFF); // 触发读取

	// 等待第一个像素数据有效
	do
	{
		reg_tmp = read_register(0x59);
		delay_us(PAW3395_TIMINGS_SRWSRR);
	} while ((reg_tmp & ((1 << PAW3395_PG_FIRST) | (1 << PAW3395_PG_VALID))) !=
			 ((1 << PAW3395_PG_FIRST) | (1 << PAW3395_PG_VALID)));

	// 读取第一个像素数据
	pFrame[35 * 2] = read_register(0x58);
	delay_us(PAW3395_TIMINGS_SRWSRR);

	// 读取剩余的像素数据 (36x36 像素阵列)
	for (uint8_t width = 0; width < 36; width++)
	{
		for (uint8_t height = 0; height < 36; height++)
		{
			// 跳过已读取的第一个像素
			if ((width == 0) && (height == 0))
				continue;

			// 等待像素数据有效
			do
			{
				reg_tmp = read_register(0x59);
				delay_us(PAW3395_TIMINGS_SRWSRR);
			} while (!((reg_tmp >> PAW3395_PG_VALID) & 0x01));

			// 计算像素在数组中的位置并读取数据
			uint16_t pixelIndex = (height * 36 + (35 - width)) * 2;
			pFrame[pixelIndex] = read_register(0x58);
			delay_us(PAW3395_TIMINGS_SRWSRR);
		}
	}

	// 恢复正常操作模式
	write_register(0x40, 0x00);
	write_register(0x50, 0x00);
	write_register(0x55, 0x00);

	// 拉高片选信号并等待退出时间
	cs_high();
	delay_125_ns(PAW3395_TIMINGS_BEXIT);
}

/**
 * 配置PAW3395传感器的DPI分辨率
 *
 * @param CPI_Num 要设置的DPI值，单位为CPI(counts per inch)
 */
void DPI_Config(uint16_t CPI_Num)
{
	uint16_t cpi_value = CPI_Num / 50; // 计算寄存器值（DPI以50为步进单位）

	// 拉低片选信号
	cs_low();
	// 等待片选信号到时钟信号的建立时间
	delay_125_ns(PAW3395_TIMINGS_NCS_SCLK);

	// 设置分辨率模式：X轴和Y轴分辨率均由X轴分辨率寄存器定义
	write_register(MOTION_CTRL, 0x00);

	// 设置X轴分辨率（低8位）
	write_register(RESOLUTION_X_LOW, (uint8_t)(cpi_value & 0xFF));
	// 设置X轴分辨率（高8位）
	write_register(RESOLUTION_X_HIGH, (uint8_t)((cpi_value >> 8) & 0xFF));

	// 应用分辨率设置
	write_register(SET_RESOLUTION, 0x01);

	// 拉高片选信号
	cs_high();
	// 等待退出时间
	delay_125_ns(PAW3395_TIMINGS_BEXIT);
}
