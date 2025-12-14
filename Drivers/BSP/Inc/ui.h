#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

    void UI_Init(void);
    void UI_Button_Update(void);   // 每次循环调用：刷新按键事件产生的状态
    void UI_Render(void);   // 每次循环调用：绘制到 OLED（u8g2 buffer + SendBuffer）

    uint8_t UI_GetSelectedQuestion(void);
    uint8_t UI_GetConfirmedQuestion(void);

#ifdef __cplusplus
}
#endif