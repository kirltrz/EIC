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

	// 利用已知参数进行校准
	Enable_Lift_Cutoff_Calibration(0x0B, 0x0C, 0x30);
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
void Motion_Burst(int16_t *dx, int16_t *dy)
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

/**
 * 设置PAW3395传感器的抬起截止高度
 * 
 * @param height_mm 抬起截止高度，只支持1mm或2mm
 *                  1 = 1mm抬起截止高度
 *                  2 = 2mm抬起截止高度
 *                  其他值将设置为默认的1mm
 * 
 * 注意：根据PAW3395手册，需要访问寄存器0x0C4E的低2位进行配置
 */
void Set_Lift_Cutoff_Height(uint8_t height_mm)
{
	uint8_t reg_value;
	uint8_t lift_setting;

	// 根据输入确定设置值
	if (height_mm == 2) {
		lift_setting = LIFT_CUTOFF_2MM; // 低2位设置为10 (2mm)
	} else {
		lift_setting = LIFT_CUTOFF_1MM; // 低2位设置为00 (1mm)
	}

	// 拉低片选信号
	cs_low();
	delay_125_ns(PAW3395_TIMINGS_NCS_SCLK);

	// 切换到Bank 0x0C以访问0x0C4E寄存器
	write_register(0x7F, 0x0C);
	delay_us(PAW3395_TIMINGS_SWR);

	// 读取当前寄存器值
	reg_value = read_register(LIFT_CUTOFF_CONFIG);
	delay_us(PAW3395_TIMINGS_SRWSRR);

	// 清除低2位，然后设置新值
	reg_value = (reg_value & ~LIFT_CUTOFF_MASK) | lift_setting;

	// 写回修改后的寄存器值
	write_register(LIFT_CUTOFF_CONFIG, reg_value);
	delay_us(PAW3395_TIMINGS_SWW);

	// 切换回Bank 0
	write_register(0x7F, 0x00);

	// 拉高片选信号
	cs_high();
	delay_125_ns(PAW3395_TIMINGS_BEXIT);
}

/**
 * 获取PAW3395传感器当前的抬起截止高度设置
 * 
 * @return 抬起截止高度值：1表示1mm，2表示2mm，0表示读取失败或未知状态
 * 
 * 注意：根据PAW3395手册，从寄存器0x0C4E的低2位读取配置
 */
uint8_t Get_Lift_Cutoff_Height(void)
{
	uint8_t reg_value;
	uint8_t lift_bits;

	// 拉低片选信号
	cs_low();
	delay_125_ns(PAW3395_TIMINGS_NCS_SCLK);

	// 切换到Bank 0x0C以访问0x0C4E寄存器
	write_register(0x7F, 0x0C);
	delay_us(PAW3395_TIMINGS_SWR);

	// 读取寄存器值
	reg_value = read_register(LIFT_CUTOFF_CONFIG);
	delay_us(PAW3395_TIMINGS_SRWSRR);

	// 切换回Bank 0
	write_register(0x7F, 0x00);

	// 拉高片选信号
	cs_high();
	delay_125_ns(PAW3395_TIMINGS_BEXIT);

	// 提取低2位
	lift_bits = reg_value & LIFT_CUTOFF_MASK;

	// 根据低2位的值返回对应的高度
	switch (lift_bits) {
		case LIFT_CUTOFF_1MM: // 00
			return 1; // 1mm
		case LIFT_CUTOFF_2MM: // 10 (0x02)
			return 2; // 2mm
		default:
			return 0; // 未知状态
	}
}

/**
 * 扩展的Motion Burst读取，包含SQUAL、RawData_Sum等完整15字节数据
 * 
 * @param data 指向motion_burst_data_t结构的指针，用于存储读取的数据
 */
void Motion_Burst_Extended(motion_burst_data_t *data)
{
	uint8_t buffer[15] = {0}; // 完整15字节的Motion Burst数据

	// 片选拉低，开始SPI通信
	cs_low();
	delay_125_ns(PAW3395_TIMINGS_NCS_SCLK);

	// 发送运动突发读取命令
	SPI_SendReceive(PAW3395_SPIREGISTER_MotionBurst);
	delay_us(PAW3395_TIMINGS_SRAD);

	// 连续读取15字节数据
	for (uint8_t i = 0; i < 15; i++)
	{
		buffer[i] = SPI_SendReceive(0x00);
	}

	// 片选拉高，结束SPI通信
	cs_high();
	delay_125_ns(PAW3395_TIMINGS_BEXIT);

	// 解析数据到结构体
	data->motion = buffer[0];
	data->observation = buffer[1];
	data->delta_x = (int16_t)(buffer[2] | (buffer[3] << 8));
	data->delta_y = (int16_t)(buffer[4] | (buffer[5] << 8));
	data->squal = buffer[6];
	data->rawdata_sum = buffer[7];
	data->maximum_rawdata = buffer[8];
	data->minimum_rawdata = buffer[9];
	data->shutter_upper = buffer[10];
	data->shutter_lower = buffer[11];
	data->reserved1 = buffer[12];
	data->reserved2 = buffer[13];
	data->squal2 = buffer[14];
}

/**
 * 执行PAW3395传感器的手动抬起截止校准
 * 严格按照PAW3395手册第7.5.1节的步骤执行
 * 
 * @return true: 校准成功, false: 校准失败
 * 
 * 注意：
 * - 校准前确保传感器已正确上电
 * - 校准期间鼠标必须放置在表面上（未抬起）
 * - 用户需要在校准过程中移动鼠标覆盖大于20英寸的距离
 */
bool Manual_Lift_Cutoff_Calibration(void)
{
	uint8_t var_mode;
	uint8_t calibration_status;
	uint8_t timeout_counter = 0;
	const uint8_t MAX_TIMEOUT = 200; // 最大等待时间

	// 拉低片选信号
	cs_low();
	delay_125_ns(PAW3395_TIMINGS_NCS_SCLK);

	// 步骤3：开始校准程序，按照手册顺序加载寄存器值
	// a. Write register 0x7F with value 0x00
	write_register(0x7F, 0x00);
	delay_us(PAW3395_TIMINGS_SWW);

	// b. Read register 0x40 and store its value into Var_Mode
	var_mode = read_register(0x40);
	delay_us(PAW3395_TIMINGS_SRWSRR);

	// c. Write register 0x40 with value 0x80
	write_register(0x40, 0x80);
	delay_us(PAW3395_TIMINGS_SWW);

	// d. Write register 0x7F with value 0x05
	write_register(0x7F, 0x05);
	delay_us(PAW3395_TIMINGS_SWW);

	// e. Write register 0x43 with value 0xE7
	write_register(0x43, 0xE7);
	delay_us(PAW3395_TIMINGS_SWW);

	// f. Write register 0x7F with value 0x04
	write_register(0x7F, 0x04);
	delay_us(PAW3395_TIMINGS_SWW);

	// g. Write register 0x40 with value 0xC0
	write_register(0x40, 0xC0);
	delay_us(PAW3395_TIMINGS_SWW);

	// h. Write register 0x41 with value 0x10
	write_register(0x41, 0x10);
	delay_us(PAW3395_TIMINGS_SWW);

	// i-p. Write registers 0x44-0x4B with value 0x0C
	for (uint8_t reg = 0x44; reg <= 0x4B; reg++) {
		write_register(reg, 0x0C);
		delay_us(PAW3395_TIMINGS_SWW);
	}

	// q. Write register 0x40 with value 0xC1 (启动校准)
	write_register(0x40, 0xC1);
	delay_us(PAW3395_TIMINGS_SWW);

	// 拉高片选信号，等待用户移动鼠标
	cs_high();
	delay_125_ns(PAW3395_TIMINGS_BEXIT);

	// 等待一段时间让用户移动鼠标（建议>20英寸距离）
	// 这里可以通过回调函数或者延时来处理
	delay_ms(5000); // 给用户5秒时间移动鼠标

	// 步骤5：停止校准过程
	cs_low();
	delay_125_ns(PAW3395_TIMINGS_NCS_SCLK);
	write_register(0x40, 0x40);
	delay_us(PAW3395_TIMINGS_SWW);

	// 步骤6：检查校准状态
	do {
		calibration_status = read_register(CALIBRATION_STATUS_REG);
		delay_us(PAW3395_TIMINGS_SRWSRR);
		
		// 检查低4位是否等于5（校准成功）
		if ((calibration_status & 0x0F) == CALIBRATION_SUCCESS) {
			break; // 校准成功
		}
		
		delay_ms(50);
		timeout_counter++;
	} while (timeout_counter < MAX_TIMEOUT);

	// 检查校准结果
	if (timeout_counter >= MAX_TIMEOUT || (calibration_status & 0x0F) != CALIBRATION_SUCCESS) {
		// 校准失败，恢复通用1mm设置
		write_register(LIFT_CUTOFF_CONFIG, UNIVERSAL_1MM_SETTING);
		delay_us(PAW3395_TIMINGS_SWW);
		
		write_register(0x7F, 0x05);
		delay_us(PAW3395_TIMINGS_SWW);
		write_register(0x43, 0xE4);
		delay_us(PAW3395_TIMINGS_SWW);
		
		write_register(0x7F, 0x00);
		delay_us(PAW3395_TIMINGS_SWW);
		write_register(0x40, var_mode);
		
		cs_high();
		delay_125_ns(PAW3395_TIMINGS_BEXIT);
		return false;
	}

	// 步骤7：校准成功，读取校准结果
	uint8_t varA = read_register(CALIBRATION_RESULT_REG);
	delay_us(PAW3395_TIMINGS_SRWSRR);

	write_register(0x7F, 0x0C);
	delay_us(PAW3395_TIMINGS_SWW);

	uint8_t varB = 0x0C;
	uint8_t varC = 0x30;

	write_register(LIFT_CUTOFF_CONFIG, UNIVERSAL_1MM_SETTING);
	delay_us(PAW3395_TIMINGS_SWW);

	write_register(0x7F, 0x05);
	delay_us(PAW3395_TIMINGS_SWW);
	write_register(0x43, 0xE4);
	delay_us(PAW3395_TIMINGS_SWW);

	write_register(0x7F, 0x00);
	delay_us(PAW3395_TIMINGS_SWW);
	write_register(0x40, var_mode);

	cs_high();
	delay_125_ns(PAW3395_TIMINGS_BEXIT);

	// 启用校准设置
	Enable_Lift_Cutoff_Calibration(varA, varB, varC);

	return true;
}

/**
 * 固件辅助手动抬起截止校准（可选功能）
 * 根据PAW3395手册第7.5.1.1节实现
 * 
 * @param varA 输出参数，校准后的VarA值
 * @param varB 输出参数，校准后的VarB值  
 * @param varC 输出参数，校准后的VarC值
 * @return true: 检测到独特表面，参数已更新; false: 未检测到独特表面，参数保持默认值
 */
bool Firmware_Aided_Calibration(uint8_t *varA, uint8_t *varB, uint8_t *varC)
{
	motion_burst_data_t burst_data;
	uint32_t squal_accumulator = 0;
	uint32_t rawdata_sum_accumulator = 0;
	uint32_t squal2_accumulator = 0;
	uint16_t sample_count = 0;

	// 设置默认值
	*varA = 0x25; // 读取的校准结果或默认值
	*varB = 0x0C;
	*varC = 0x30;

	// 步骤1：收集至少3000个样本
	while (sample_count < MIN_MOTION_SAMPLES) {
		Motion_Burst_Extended(&burst_data);
		
		// 只在有运动时累加数据
		if (burst_data.motion & 0x80) { // 检查运动标志位
			squal_accumulator += burst_data.squal;
			rawdata_sum_accumulator += burst_data.rawdata_sum;
			squal2_accumulator += burst_data.squal2;
			sample_count++;
		}
		
		delay_ms(1); // 小延时避免过快采样
	}

	// 计算平均值
	uint16_t squal_avg = squal_accumulator / sample_count;
	uint16_t rawdata_sum_avg = rawdata_sum_accumulator / sample_count;
	uint16_t squal2_avg = squal2_accumulator / sample_count;

	// 步骤2：确定SQUAL阈值
	uint8_t squal_th = (rawdata_sum_avg < 48) ? 23 : 30;

	// 步骤3：计算SQUAL比率
	uint16_t squal_ratio = 0;
	if (squal_avg > 0) {
		squal_ratio = (squal2_avg * 100) / squal_avg;
	}

	// 步骤4：检查是否检测到独特表面
	if ((squal_ratio < squal_th) && (rawdata_sum_avg < 68)) {
		// 检测到独特表面，更新参数
		*varA = 0x25;
		*varB = 0x0C;
		*varC = 0x2D;
		return true;
	}

	// 未检测到独特表面，保持默认值
	return false;
}

/**
 * 启用抬起截止校准寄存器设置
 * 根据PAW3395手册第7.5.2节实现
 * 
 * @param varA 校准参数A
 * @param varB 校准参数B
 * @param varC 校准参数C
 */
void Enable_Lift_Cutoff_Calibration(uint8_t varA, uint8_t varB, uint8_t varC)
{
	cs_low();
	delay_125_ns(PAW3395_TIMINGS_NCS_SCLK);

	// 按照手册步骤设置寄存器
	write_register(0x7F, 0x0C);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x41, varA);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x43, varC);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x44, varB);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x4E, 0x08);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x5A, 0x0D);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x5B, 0x05);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x7F, 0x05);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x6E, 0x0F);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x7F, 0x09);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x71, 0x0C);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x7F, 0x00);

	cs_high();
	delay_125_ns(PAW3395_TIMINGS_BEXIT);
}

/**
 * 禁用抬起截止校准寄存器设置，恢复默认通用1mm设置
 * 根据PAW3395手册第7.5.3节实现
 */
void Disable_Lift_Cutoff_Calibration(void)
{
	cs_low();
	delay_125_ns(PAW3395_TIMINGS_NCS_SCLK);

	// 按照手册步骤恢复默认设置
	write_register(0x7F, 0x0C);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x41, 0x25);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x43, 0x2D);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x44, 0x0C);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x4A, 0x10);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x4B, 0x0C);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x4C, 0x40);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x4E, 0x08);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x53, 0x16);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x54, 0x1A);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x55, 0x18);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x56, 0x14);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x5A, 0x0D);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x5B, 0x05);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x5F, 0x1E);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x66, 0x30);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x7F, 0x05);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x6E, 0x0F);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x7F, 0x09);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x71, 0x0C);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x72, 0x07);
	delay_us(PAW3395_TIMINGS_SWW);
	
	write_register(0x7F, 0x00);

	cs_high();
	delay_125_ns(PAW3395_TIMINGS_BEXIT);
}
