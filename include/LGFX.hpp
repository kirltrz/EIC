#include <LovyanGFX.hpp>

class LGFX : public lgfx::LGFX_Device
{
    lgfx::Panel_ST7789 _panel_instance;
    lgfx::Bus_SPI _bus_instance;
    lgfx::Touch_XPT2046 _touch_instance;

public:
    // 创建构造函数，在此进行各种设定。
    // 如果更改了类名，构造函数也请指定相同的名称。
    LGFX(void)
    {
        {                                      // 进行总线控制的设定。
            auto cfg = _bus_instance.config(); // 取得总线设定用的结构体。

            // 配置Spi总线
            cfg.spi_host = SPI2_HOST; // 选择要使用的SPI ESP32-S2，C3: spi_host or spi_host/ESP32: VSPI_HOST or HSPI_HOST
            // ※ 随着ESP-IDF版本升级，VSPI_HOST、HSPI_HOST的记述将被不推荐，出现错误时请使用SPI2_HOST、SPI3_HOST代替。
            cfg.spi_mode = 0;                  // 设定SPI通信模式 (0 ~ 3)
            cfg.freq_write = 27000000;         // 发送时的SPI时钟 (最大80MHz，80MHz除以整数后的值)
            cfg.freq_read = 20000000;          // 接收时的spi时钟
            cfg.spi_3wire = true;              // 用mosi引脚进行接收时设定为true
            cfg.use_lock = true;               // 使用事务锁时设定true
            cfg.dma_channel = SPI_DMA_CH_AUTO; // 设置要使用的DMA通道 (0 = 不使用DMA/1 = 1ch / 2 = ch / SPI_DMA_CH_AUTO = 自动设置)
            // ※ 随着ESP-IDF版本升级，DMA频道推荐SPI_DMA_CH_AUTO (自动设定)。1ch，2ch的指定不推荐。
            cfg.pin_sclk = 15; // 设定Spi的sclk引脚编号
            cfg.pin_mosi = 7;  // 设定Spi的mosi引脚编号
            cfg.pin_miso = 6;  // 设定SPI的MISO引脚编号 (-1 = disable)
            cfg.pin_dc = 18;   // 设置SPI的D/C引脚编号 (-1 = disable)
                               // 使用与sd卡共同的SPI总线时，不要省略MISO，请务必设定。
            _bus_instance.config(cfg);              // 在总线上反映设定值。
            _panel_instance.setBus(&_bus_instance); // 将总线设置在面板上。

        }

        {                                        // 进行显示面板控制的设定。
            auto cfg = _panel_instance.config(); // 取得显示面板设定用的结构体。

            cfg.pin_cs = 8;    // CS所连接的引脚编号 (-1 = disable)
            cfg.pin_rst = 3;   // RST所连接的引脚编号 (-1 = disable)
            cfg.pin_busy = -1; // BUSY所连接的引脚编号 (-1 = disable)

            // ※ 以下的设定值每个面板都设定了一般的初始值，不明的项目请注释出来试试。

            cfg.panel_width = 240;    // 实际可显示的宽度
            cfg.panel_height = 320;   // 实际可显示的高度
            cfg.offset_x = 0;         // 面板x方向偏移量
            cfg.offset_y = 0;         // 面板的y方向偏移量
            cfg.offset_rotation = 1;  // 旋转方向的值的偏移0 ~ 7 (4 ~ 7为上下反转)
            cfg.dummy_read_pixel = 8; // 像素读取前虚拟引线的位数
            cfg.dummy_read_bits = 1;  // 非像素数据读取前的虚拟读取位数
            cfg.readable = true;      // 可读取数据时设定为true
            cfg.invert = true;        // 面板的明暗反转时设定为true
            cfg.rgb_order = false;    // 面板的红色和蓝色交换时设定为true
            cfg.dlen_16bit = false; // 在16bit并行或SPI中以16bit为单位发送数据长度的面板的情况下设定为true
            cfg.bus_shared = false; // 与sd卡共享总线时设定为true (通过drawjpg file等进行总线控制

            _panel_instance.config(cfg);
        }

        { // 进行触摸屏控制的设定。(不需要的话删除)
            auto cfg = _touch_instance.config();
            cfg.x_min = 0;           // 从触摸屏获得的最小x值 (原始值)
            cfg.x_max = 239;         // 从触摸屏获得的最大x值 (原始值)
            cfg.y_min = 0;           // 从触摸屏获得的最小y值 (原始值)
            cfg.y_max = 319;         // 从触摸屏获得的最大y值 (原始值)
            cfg.pin_int = 17;        // 连接有Int的引脚编号
            cfg.bus_shared = true;   // 使用与画面共通的总线时设定true
            cfg.offset_rotation = 0; // 显示和触摸方向不一致时的调整0 ~ 7的值设定

            // 对于Spi连接
            cfg.spi_host = SPI2_HOST; // 选择要使用的SPI (HSPI_HOST or VSPI_HOST)
            cfg.freq = 1000000;       // 设定Spi时钟
            cfg.pin_sclk = 15;        // Sclk所连接的引脚编号
            cfg.pin_mosi = 7;         // 连接有Mosi的引脚编号
            cfg.pin_miso = 6;         // 连接Miso的引脚编号
            cfg.pin_cs = 16;          //   Cs所连接的引脚编号

            _touch_instance.config(cfg);
            _panel_instance.setTouch(&_touch_instance); // 将触摸屏设置在面板上。
        }
        setPanel(&_panel_instance); // 设置要使用的面板。
    }
};