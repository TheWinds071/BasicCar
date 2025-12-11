#include "key_driver.h"

/**
 * @brief 初始化按键结构体
 */
void Key_Init(Key_t* key, GPIO_TypeDef* port, uint16_t pin, GPIO_PinState active_level) {
    key->port = port;
    key->pin = pin;
    key->active_level = active_level;
    key->state = KEY_STATE_IDLE;
    key->start_tick = 0;
}

/**
 * @brief 读取物理引脚状态
 * @return 1: 按键按下 (Active), 0: 按键松开
 */
static uint8_t Key_ReadPin(Key_t* key) {
    if (HAL_GPIO_ReadPin(key->port, key->pin) == key->active_level) {
        return 1;
    }
    return 0;
}

/**
 * @brief 按键扫描状态机 (需在主循环中周期性调用)
 */
KeyEvent_t Key_Scan(Key_t* key) {
    KeyEvent_t event = KEY_EVENT_NONE;
    uint32_t current_tick = HAL_GetTick(); // 获取系统滴答 (ms)

    switch (key->state) {
        // --- 状态 1: 空闲 ---
        case KEY_STATE_IDLE:
            if (Key_ReadPin(key)) {
                key->state = KEY_STATE_DEBOUNCE;
                key->start_tick = current_tick; // 记录开始消抖的时间
            }
            break;

        // --- 状态 2: 消抖 ---
        case KEY_STATE_DEBOUNCE:
            // 如果消抖时间内按键松开了，说明是抖动/噪声，回退到IDLE
            if (!Key_ReadPin(key)) {
                key->state = KEY_STATE_IDLE;
            }
            // 消抖时间到达
            else if ((current_tick - key->start_tick) >= DEBOUNCE_TICKS) {
                key->state = KEY_STATE_PRESSED;
                // 注意：这里不更新start_tick，以便从最初按下开始计算长按
            }
            break;

        // --- 状态 3: 已确认按下 (检测长按或释放) ---
        case KEY_STATE_PRESSED:
            // 1. 如果松开，判定为"单击"
            if (!Key_ReadPin(key)) {
                key->state = KEY_STATE_IDLE;
                event = KEY_EVENT_CLICK;
            }
            // 2. 如果一直按着，检查是否达到长按阈值
            else if ((current_tick - key->start_tick) >= LONG_PRESS_TICKS) {
                key->state = KEY_STATE_LONG_PRESS;
                event = KEY_EVENT_LONG_PRESS;
            }
            break;

        // --- 状态 4: 长按保持中 ---
        case KEY_STATE_LONG_PRESS:
            // 等待按键松开，防止松开瞬间再次触发单击
            if (!Key_ReadPin(key)) {
                key->state = KEY_STATE_IDLE;
                event = KEY_EVENT_RELEASE; // 可选：返回释放事件
            }
            break;
    }

    return event;
}