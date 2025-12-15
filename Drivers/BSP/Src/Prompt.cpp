#include "Prompt.hpp"

extern "C" {
#include "main.h"
#include "gpio.h"
#include "stm32h7xx_hal.h"
}

void Prompt::on()
{
    // 蜂鸣器 PC3：1 响
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
    // 灯 PC1(LED_G)：0 亮（低电平点亮）
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
}

void Prompt::off()
{
    // 蜂鸣器 0 不响
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
    // 灯 1 灭
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
}

void Prompt::init()
{
    s_active = false;
    s_offTime = 0;
    off();
}

void Prompt::once(uint32_t duration_ms)
{
    const uint32_t now = HAL_GetTick();
    on();
    s_active = true;
    s_offTime = now + duration_ms; // 允许重触发刷新截止时间
}

void Prompt::tick(uint32_t now_ms)
{
    if (!s_active) return;

    // 处理 HAL_GetTick() 回绕：用有符号差判断
    if (static_cast<int32_t>(now_ms - s_offTime) >= 0) {
        off();
        s_active = false;
    }
}