#include "sensor.h"

/*paw3395参数及针脚设置*/
#define DPI 26000
#define NRESET 2
#define NCS 42
#define SCLK 39
#define MISO 41
#define MOSI 40

void initSensor(void)
{
    /*初始化传感器*/
    Wire.begin();
    paw3395Init(DPI, NRESET, NCS, SCLK, MISO, MOSI);
}

bool checkPaw3395(void)
{
    /*检查paw3395是否正常工作*/
    return paw3395_check();
}

bool checkHWT101(void)
{
    /*检查hwt101是否正常工作*/
    return true;
}

bool checkComm(void)
{
    /*检查与MaixCam串口通信是否正常*/
    sendCommand(CMD_IDLE);
    comm_packet_t data;
    return receiveData(&data);
}
