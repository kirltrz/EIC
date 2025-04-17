#include"ZDTX42V2.h"

#define DEFAULT_SPEED 100.0f
#define DEFAULT_ACC 100
struct POS{
    float x;
    float y;
    float yaw;
};
void moveTo(POS pos, float speed=DEFAULT_SPEED, int acc=DEFAULT_ACC, int dec=DEFAULT_ACC);
void moveTask(void*pvParameters);
bool arrived(void);//返回true表示到达目标点