#include "arm.h"

FSUS_Protocol protocol_ch1(&Serial1, 115200);
FSUS_Servo servo0(0,&protocol_ch1);
FSUS_Servo servo1(1,&protocol_ch1);
FSUS_Servo servo2(2,&protocol_ch1);
void initArm(void)
{
    /*初始化机械臂*/
}

void moveArm(int x, int y, int z)
{
    /*移动机械臂*/
}

void catchFromTurntable(int taskcode[3])
{
    /*从转盘抓取*/
}

void putToGround(int taskcode[3])
{
    /*放置到地面*/
}

void catchFromGround(int taskcode[3])
{
    /*从地面抓取*/
}

void putToMaterial(int taskcode[3])
{
    /*码垛*/
}

