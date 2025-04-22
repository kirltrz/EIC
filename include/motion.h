#include"ZDTX42V2.h"

#define DEFAULT_SPEED 100.0f
#define DEFAULT_ACC 100

extern ZDTX42V2* motor;
struct POS{
    float x;
    float y;
    float yaw;
};
void initMotor(void);
void moveTo(POS pos, float speed=DEFAULT_SPEED, int acc=DEFAULT_ACC, int dec=DEFAULT_ACC);
void moveTask(void*pvParameters);
bool arrived(void);//返回true表示到达目标点
void stopMotion(void);//停止运动并释放电机控制
void forceStopAllMotors(void);//强制停止所有电机，确保速度为0