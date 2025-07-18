#include "HWT101.h"
#include "string.h"

HWT101CLASS ::HWT101CLASS()
{
	devAddr = 0x50;
}

void HWT101CLASS::writeRegister(unsigned char deviceAddr, unsigned char addressToWrite, unsigned char bytesToRead, char *dataToWrite)
{
	Wire.beginTransmission(deviceAddr);
	Wire.write(addressToWrite);
	for (int i = 0; i < bytesToRead; i++)
		Wire.write(dataToWrite[i]);
	Wire.endTransmission();
}

void HWT101CLASS::readRegisters(unsigned char deviceAddr, unsigned char addressToRead, unsigned char bytesToRead, char *dest)
{
	Wire.beginTransmission(deviceAddr);
	Wire.write(addressToRead);
	Wire.endTransmission(false);

	Wire.requestFrom(deviceAddr, bytesToRead);

	while (Wire.available() < bytesToRead)
		; // 等待数据

	for (int x = 0; x < bytesToRead; x++)
		dest[x] = Wire.read();
}

float HWT101CLASS::getZ(float *ptr)
{
	readRegisters(devAddr, YAW, 2, (char *)&rawYaw);
	float angle = round(((float)rawYaw / 32768 * 180) * 100) / 100;//保留一位小数，因为HWT101的精度为0.1度，更小的角度会引入噪声
	if (ptr != nullptr)
		*ptr = angle;
	return angle;
}
/**
 * @brief 获取Z轴角速度，单位°/s
 * @param ptr 输出参数，可选
 * @return 角速度值
 */
float HWT101CLASS::getAngularVelocityZ(float *ptr){
	int16_t angularVelocityZ;
	readRegisters(devAddr, GZ, 2, (char *)&angularVelocityZ);

	// 角速度Z=GZ[15:0]/32768*2000°/s
	float velocity = ((float)angularVelocityZ / 32768.0f * 2000.0f);
	
	if (ptr != nullptr)
		*ptr = velocity;
	return velocity;
}
void HWT101CLASS::toZero(void)
{
	writeRegister(devAddr, UNLOCK, 2, unlock);
	delay(1);
	writeRegister(devAddr, CALIYAW, 2, tozero);
	delay(1);
	writeRegister(devAddr, SAVE, 2, lock);
}

bool HWT101CLASS::setOutputRate(float hz)
{
	uint8_t rate;
	
	// 将Hz值映射到寄存器值
	if (hz <= 0.2f)
		rate = 0x01;      // 0.2Hz
	else if (hz <= 0.5f)
		rate = 0x02;      // 0.5Hz
	else if (hz <= 1.0f)
		rate = 0x03;      // 1Hz
	else if (hz <= 2.0f)
		rate = 0x04;      // 2Hz
	else if (hz <= 5.0f)
		rate = 0x05;      // 5Hz
	else if (hz <= 10.0f)
		rate = 0x06;      // 10Hz
	else if (hz <= 20.0f)
		rate = 0x07;      // 20Hz
	else if (hz <= 50.0f)
		rate = 0x08;      // 50Hz
	else if (hz <= 100.0f)
		rate = 0x09;      // 100Hz
	else if (hz <= 200.0f)
		rate = 0x0B;      // 200Hz
	else if (hz <= 500.0f)
		rate = 0x0C;      // 500Hz
	else
		rate = 0x0D;      // 1000Hz
	
	// 准备数据，采用小端序（低字节在前）
	char data[2] = {(char)rate, 0x00};
	
	// 解锁、写入速率、保存配置
	writeRegister(devAddr, UNLOCK, 2, unlock);
	delay(1);
	writeRegister(devAddr, RRATE, 2, data);
	delay(1);
	writeRegister(devAddr, SAVE, 2, lock);
	
	return true;
}

HWT101CLASS HWT101 = HWT101CLASS();
