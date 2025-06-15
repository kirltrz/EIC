SET(SOURCES ui_startScr.c
    ui_taskcodeScr.c
    ui.c
    ui_comp_hook.c
    ui_helpers.c
    ui_events.cpp
    fonts/ui_font_bigchinese.c
    fonts/ui_font_chinese.c
    fonts/ui_font_taskcode.c)

add_library(ui ${SOURCES})
