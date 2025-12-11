#include "button.hpp"

// 构造函数
Button::Button(GPIO_TypeDef* port, uint16_t pin, bool active_low)
    : _port(port), _pin(pin), _active_low(active_low),
      _state(State::Idle), _start_tick(0)
{
}

// 注册单击回调
void Button::attachClick(ButtonCallback cb) {
    _onClick = cb;
}

// 注册长按回调
void Button::attachLongPress(ButtonCallback cb) {
    _onLongPress = cb;
}

// 读取物理引脚状态
bool Button::isRawPressed() const {
    GPIO_PinState state = HAL_GPIO_ReadPin(_port, _pin);
    // 如果是低电平有效，读取到 RESET(0) 算按下
    return _active_low ? (state == GPIO_PIN_RESET) : (state == GPIO_PIN_SET);
}

// 状态机核心逻辑
void Button::scan() {
    uint32_t current_tick = HAL_GetTick();

    switch (_state) {
        // --- 空闲状态 ---
        case State::Idle:
            if (isRawPressed()) {
                _state = State::Debounce;
                _start_tick = current_tick;
            }
            break;

        // --- 消抖状态 ---
        case State::Debounce:
            if (!isRawPressed()) {
                // 假的，回退到空闲
                _state = State::Idle;
            } else if ((current_tick - _start_tick) >= DEBOUNCE_TIME) {
                // 真的按下了，进入 Pressed
                _state = State::Pressed;
                // 注意：这里不更新 _start_tick，因为长按计时要从最初按下算起
            }
            break;

        // --- 已按下状态 (等待松开或长按) ---
        case State::Pressed:
            if (!isRawPressed()) {
                // 还没到长按时间就松开了 -> 判定为【单击】
                if (_onClick != nullptr) _onClick(); // 执行回调
                _state = State::Idle;
            }
            else if ((current_tick - _start_tick) >= LONG_PRESS_TIME) {
                // 超过长按阈值 -> 判定为【长按】
                if (_onLongPress != nullptr) _onLongPress(); // 执行回调
                _state = State::LongPressWait;
            }
            break;

        // --- 长按保持状态 ---
        case State::LongPressWait:
            // 等待直到用户松手，防止松手瞬间误触发单击
            if (!isRawPressed()) {
                _state = State::Idle;
            }
            break;
    }
}