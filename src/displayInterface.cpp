#include "displayInterface.h"
#include "config.h"
#include "sensor.h"
#include "motion.h"
#include "arm.h"
#include "ota.h"

LGFX display;

uint16_t calibrateData[8] = {3912, 161, 3928, 3772, 219, 188, 222, 3768}; // 触摸屏校准数据
const uint16_t screenWidth = 320, screenHeight = 240;
uint16_t buf[screenWidth * screenHeight * sizeof(uint16_t) / 10];

void lvglTask(void *pvParameters)
{
    while (true)
    {
        lv_timer_handler();
        wait(5);
    }
}

static void lv_update_task(lv_timer_t *timer)
{
    // 每50ms更新一次UI内容
    static uint32_t last_200ms_update = 0;
    static uint32_t last_1s_update = 0;
    uint32_t current_time = millis();
    char buf[10][16]; // 静态缓冲区

    // 更新OTA UI
    updateOTAUI();

    // 每200ms更新一次
    if (current_time - last_200ms_update >= 200)
    {
        last_200ms_update = current_time;

        global_position_t globalPos;
        getGlobalPosition(&globalPos);

        sprintf(buf[0], "%.2f", globalPos.x);
        sprintf(buf[1], "%.2f", globalPos.y);
        sprintf(buf[2], "%.2f", globalPos.continuousYaw);

        lv_label_set_text(ui_currentX, buf[0]);
        lv_label_set_text(ui_currentY, buf[1]);
        lv_label_set_text(ui_currentYaw, buf[2]);
    }

    // 每5s更新一次电压信息（耗时操作）,如果电压为700mV则代表数据获取异常或者第一次更新则获取一次
    /*static int voltage = 700;
    if (voltage == 700 || current_time - last_1s_update >= 1000)
    {
        last_1s_update = current_time;

        //在此调用获取数据的函数会阻塞任务导致CPU占用极高，降低更新频率
        voltage = motor->getVoltage();
        //sprintf(buf[3], "%d", voltage);
        //lv_label_set_text(ui_voltage, buf[3]);//不显示具体电压值，只显示电量条和百分比

        // 计算电池百分比 (9V-12.7V 映射到 0-100%)
        int percentage = (voltage - 9000) * 100 / (12700 - 9000);
        if (percentage > 100)
            percentage = 100;
        if (percentage < 0)
            percentage = 0;

        // 更新电池条百分比
        if (voltage != 700) // 700为电机压降补偿值，出现此值代表数据获取异常
        {
            lv_bar_set_value(ui_batteryPercentBar, percentage, LV_ANIM_ON);
            // 更新电池条文字
            char percent_text[8];
            sprintf(percent_text, "%d%%", percentage);
            lv_label_set_text(ui_Label52, percent_text);

            // 根据电量设置颜色
            if (percentage >= 80)
            { // 高电量
                lv_obj_set_style_bg_color(ui_batteryPercentBar, lv_color_hex(0xF8FFFB), LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(ui_batteryPercentBar, lv_color_hex(0x54DC4E), LV_PART_INDICATOR | LV_STATE_DEFAULT);
            }
            else if (percentage >= 60)
            { // 中高电量
                lv_obj_set_style_bg_color(ui_batteryPercentBar, lv_color_hex(0xF9FBE7), LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(ui_batteryPercentBar, lv_color_hex(0xCDDC39), LV_PART_INDICATOR | LV_STATE_DEFAULT);
            }
            else if (percentage >= 40)
            { // 中等电量
                lv_obj_set_style_bg_color(ui_batteryPercentBar, lv_color_hex(0xFFFDE7), LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(ui_batteryPercentBar, lv_color_hex(0xFFD54F), LV_PART_INDICATOR | LV_STATE_DEFAULT);
            }
            else if (percentage >= 20)
            { // 低电量
                lv_obj_set_style_bg_color(ui_batteryPercentBar, lv_color_hex(0xFFF3E0), LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(ui_batteryPercentBar, lv_color_hex(0xFFB74D), LV_PART_INDICATOR | LV_STATE_DEFAULT);
            }
            else
            { // 极低电量
                lv_obj_set_style_bg_color(ui_batteryPercentBar, lv_color_hex(0xFFEBEE), LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(ui_batteryPercentBar, lv_color_hex(0xEF9A9A), LV_PART_INDICATOR | LV_STATE_DEFAULT);
            }
        }else{//如果电压为700则代表数据获取异常，显示灰色，百分比不变
            lv_obj_set_style_bg_color(ui_batteryPercentBar, lv_color_hex(0xF5F5F5), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(ui_batteryPercentBar, lv_color_hex(0xB0BEC5), LV_PART_INDICATOR | LV_STATE_DEFAULT);
        }
    }*/

}

void flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    int32_t x1 = area->x1;
    int32_t y1 = area->y1;

    display.startWrite();                // 开始写入操作
    display.setAddrWindow(x1, y1, w, h); // 设置绘图窗口

    display.writePixels((uint16_t *)px_map, w * h, true); // 写入数据

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

    /*创建lvgl周期任务用于更新UI*/
    (void)lv_timer_create(lv_update_task, 50, NULL);
}