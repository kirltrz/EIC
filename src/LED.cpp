#include "LED.h"

#define LED_PIN 1
#define LED_HIGH 255
#define LED_MIDDLE 20
#define LED_OFF 0
void initLED(void)
{
    pinMode(LED_PIN, OUTPUT);
}
void LED(int level)
{
    switch (level)
    {
    case 0:
        analogWrite(LED_PIN, LED_OFF);
        break;
    case 1:
        analogWrite(LED_PIN, LED_HIGH);
        break;
    case 2:
        analogWrite(LED_PIN, LED_MIDDLE);
        break;

    default:
        break;
    }
}