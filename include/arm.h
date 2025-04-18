#include "FashionStar_UartServo.h"

/*
@brief 机械臂运动函数
*/
void arm_ScanQRcode();
void arm_catchFromTurntable(int taskcode[3]);
void arm_putToGround(int taskcode[3], int circleOffset[3][2]/*传出当前色环偏移量*/);
void arm_catchFromGround(int taskcode[3], const int circleOffset[3][2]/*传入当前色环偏移量*/);
void arm_putToMaterial(int taskcode[3]);
