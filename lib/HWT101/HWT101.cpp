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
	float angle = round(((float)rawYaw / 32768 * 180) * 1000) / 1000;
	if (ptr != nullptr)
		*ptr = angle;
	return angle;
}
void HWT101CLASS::toZero(void)
{
	writeRegister(devAddr, UNLOCK, 2, unlock);
	delay(1);
	writeRegister(devAddr, CALIYAW, 2, tozero);
	delay(1);
	writeRegister(devAddr, SAVE, 2, lock);
}

HWT101CLASS HWT101 = HWT101CLASS();
