#include "displayInterface.h"
#include "config.h"

LGFX display;

uint16_t calibrateData[8] = {3912, 161, 3928, 3772, 219, 188, 222, 3768}; // 触摸屏校准数据
const uint16_t screenWidth = 320, screenHeight = 240;
uint16_t buf[screenWidth * screenHeight / 10];

void lvglTask(void *pvParameters)
{
    while (true)
    {
        lv_timer_handler();
        vTaskDelay(5);
    }
}
void flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    int32_t x1 = area->x1;
    int32_t y1 = area->y1;

    display.startWrite();                // 开始写入操作
    display.setAddrWindow(x1, y1, w, h); // 设置绘图窗口

    // uint16_t *data = (uint16_t *)px_map;
    display.writePixels((uint16_t *)px_map, w * h, true);

    display.endWrite(); // 结束写入操作

    lv_display_flush_ready(disp);
}
void my_touchpad_read(lv_indev_t *indev, lv_indev_data_t *data)
{
    int32_t x, y;
    bool touched = display.getTouch(&x, &y);

    if (!touched)
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
    else
    {
        data->state = LV_INDEV_STATE_PRESSED;
        data->point.x = x;
        data->point.y = y;
    }
}
static uint32_t my_tick_get_cb(void) { return millis(); }

void initGUI(void)
{
    /*初始化屏幕*/
    display.init();                           // 初始化屏幕
    display.setTouchCalibrate(calibrateData); // 校准触摸
    display.fillScreen(TFT_BLACK);            // 屏幕填充黑色

    /*初始化lvgl*/
    lv_init();                      // 初始化lvgl
    lv_tick_set_cb(my_tick_get_cb); // 设置lvgl心跳回调

    lv_display_t *disp = lv_display_create(screenWidth, screenHeight);                    // 创建lvgl显示设备
    lv_display_set_flush_cb(disp, flush_cb);                                              // 设置刷屏函数
    lv_display_set_buffers(disp, buf, NULL, sizeof(buf), LV_DISPLAY_RENDER_MODE_PARTIAL); // 设置缓冲区

    lv_indev_t *indev = lv_indev_create();           // 创建lvgl输入设备
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER); // 设置输入设备类型
    lv_indev_set_read_cb(indev, my_touchpad_read);   // 设置输入设备读取函数

    /*加载UI*/
    ui_init();
}