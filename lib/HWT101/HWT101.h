#ifndef HWT101_h
#define HWT101_h

#include <stdint.h>
#include <Wire.h>

#define SAVE    0x00
#define YAW     0x3F
#define UNLOCK  0x69
#define CALIYAW 0x76

class HWT101CLASS
{
public:
	int16_t rawYaw;

	HWT101CLASS();

	float getZ(float *ptr = nullptr);
	void toZero(void);

private:
	unsigned char devAddr;
	char unlock[2] = {0x88, 0xB5};
	char tozero[2] = {0x00, 0x00};
	char lock[2] = {0x00, 0x00};
	void readRegisters(unsigned char deviceAddr, unsigned char addressToRead, unsigned char bytesToRead, char *dest);
	void writeRegister(unsigned char deviceAddr, unsigned char addressToWrite, unsigned char bytesToRead, char *dataToWrite);
};
extern HWT101CLASS HWT101;

#endif