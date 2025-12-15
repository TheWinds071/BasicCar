#include "ui.h"

#include "main.h"
#include "button.hpp"
#include "u8g2.h"
#include "OLED.h"     // drawFloatPrec()

#include <cstdio>
#include <cstring>

extern u8g2_t u8g2;
extern float User_YPR[3];

// UI state
static volatile uint8_t g_selected_q = 1;   // 1..4
static volatile uint8_t g_confirmed_q = 0;  // 0=none

// Keys: Up=PE6(Button1), Down=PE4(Button2), OK=PE5(Button3)
static Button btnUp(Button2_GPIO_Port, Button2_Pin, true);
static Button btnDown(Button1_GPIO_Port, Button1_Pin, true);
static Button btnOk(Button3_GPIO_Port, Button3_Pin, true);

static void onUpClick() {
    g_selected_q = (g_selected_q <= 1) ? 4 : static_cast<uint8_t>(g_selected_q - 1);
}
static void onDownClick() {
    g_selected_q = (g_selected_q >= 4) ? 1 : static_cast<uint8_t>(g_selected_q + 1);
}
static void onOkClick() {
    g_confirmed_q = g_selected_q;
}

void UI_Init(void) {
    btnUp.attachClick(onUpClick);
    btnDown.attachClick(onDownClick);
    btnOk.attachClick(onOkClick);
}

void UI_Button_Update(void) {
    btnUp.scan();
    btnDown.scan();
    btnOk.scan();
}

static void draw_top_bar(void) {
    // 顶部反显标题栏
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawBox(&u8g2, 0, 0, 128, 12);

    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawStr(&u8g2, 2, 10, "BasicCar UI");

    char buf[20];
    if (g_confirmed_q == 0) {
        snprintf(buf, sizeof(buf), "Q%d", (int)g_selected_q);
    } else if (g_confirmed_q == g_selected_q) {
        snprintf(buf, sizeof(buf), "Q%d OK", (int)g_selected_q);
    } else {
        snprintf(buf, sizeof(buf), "Q%d>%d", (int)g_selected_q, (int)g_confirmed_q);
    }

    int x = 128 - (int)(6 * (int)strlen(buf)) - 2;
    if (x < 60) x = 60;
    u8g2_DrawStr(&u8g2, (u8g2_uint_t)x, 10, buf);

    // 恢复为正常绘制颜色
    u8g2_SetDrawColor(&u8g2, 1);
}

static void draw_question_selector(void) {
    // Q1..Q4 选择条：选中反显；确认画外框
    const uint8_t y = 14;
    const uint8_t h = 12;
    const uint8_t w = 30;
    const uint8_t gap = 2;

    for (uint8_t i = 1; i <= 4; i++) {
        auto x = static_cast<uint8_t>((i - 1) * (w + gap));
        if (x + w > 128) break;

        if (g_confirmed_q == i) {
            u8g2_DrawFrame(&u8g2, x, y, w, h);
        }

        if (g_selected_q == i) {
            u8g2_DrawBox(&u8g2, x + 1, y + 1, w - 2, h - 2);
            u8g2_SetDrawColor(&u8g2, 0);
        } else {
            u8g2_SetDrawColor(&u8g2, 1);
        }

        char label[4];
        snprintf(label, sizeof(label), "Q%u", (unsigned)i);
        u8g2_DrawStr(&u8g2, x + 7, y + 10, label);

        u8g2_SetDrawColor(&u8g2, 1);
    }

    u8g2_DrawHLine(&u8g2, 0, 28, 128);
}

static void draw_ypr_table(float yaw, float pitch, float roll) {
    // 行 baseline：40/52/64（最后一行贴边，用 6x12 字体一般还行）
    constexpr uint8_t y1 = 40;
    constexpr uint8_t y2 = 52;
    constexpr uint8_t y3 = 64;

    // 左侧标签
    u8g2_DrawStr(&u8g2, 0,  y1, "Yaw:");
    u8g2_DrawStr(&u8g2, 0,  y2, "Pit:");
    u8g2_DrawStr(&u8g2, 0,  y3, "Rol:");

    // 右侧数值（用你的 drawFloatPrec）
    // x=40 给足够空间
    drawFloatPrec(&u8g2, 40, y1, yaw,   2);
    drawFloatPrec(&u8g2, 40, y2, pitch, 2);
    drawFloatPrec(&u8g2, 40, y3, roll,  2);
}

void UI_Render(void) {
    // 本地拷贝一次，减少中断更新撕裂
    float yaw = User_YPR[0];
    float pit = User_YPR[1];
    float rol = User_YPR[2];

    u8g2_ClearBuffer(&u8g2);

    draw_top_bar();
    draw_question_selector();
    draw_ypr_table(yaw, pit, rol);

    u8g2_SendBuffer(&u8g2);
}

uint8_t UI_GetSelectedQuestion(void) { return g_selected_q; }
uint8_t UI_GetConfirmedQuestion(void) { return g_confirmed_q; }