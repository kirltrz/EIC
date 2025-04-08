SET(SOURCES ui_Screen1.c
    ui.c
    ui_comp_hook.c
    ui_helpers.c
    fonts/ui_font_bigchinese.c
    fonts/ui_font_chinese.c
    fonts/ui_font_taskcode.c)

add_library(ui ${SOURCES})
