/*
* Button.hpp
 * 非阻塞状态机按键驱动 (C++版)
 */

#pragma once

#include "main.h" // 确保包含 HAL 库定义 (stm32f1xx_hal.h, etc.)

// 定义回调函数指针类型 (无参数，无返回值)
typedef void (*ButtonCallback)(void);

class Button {
public:
    // 按键事件类型
    enum class Event {
        None,
        Click,
        LongPress
    };

    /**
     * @brief 构造函数
     * @param port GPIO端口 (如 GPIOA)
     * @param pin  GPIO引脚 (如 GPIO_PIN_0)
     * @param active_low true:低电平有效(按下接地), false:高电平有效
     */
    Button(GPIO_TypeDef* port, uint16_t pin, bool active_low = true);

    /**
     * @brief 注册单击回调函数
     */
    void attachClick(ButtonCallback cb);

    /**
     * @brief 注册长按回调函数
     */
    void attachLongPress(ButtonCallback cb);

    /**
     * @brief 核心扫描函数，需在主循环中周期性调用
     */
    void scan();

private:
    // 内部状态
    enum class State {
        Idle,
        Debounce,
        Pressed,
        LongPressWait
    };

    // 硬件参数
    GPIO_TypeDef* _port;
    uint16_t      _pin;
    bool          _active_low;

    // 运行变量
    State         _state;
    uint32_t      _start_tick;

    // 回调函数存储
    ButtonCallback _onClick = nullptr;
    ButtonCallback _onLongPress = nullptr;

    // 配置参数 (ms)
    static constexpr uint32_t DEBOUNCE_TIME = 20;
    static constexpr uint32_t LONG_PRESS_TIME = 1000;

    // 内部辅助函数
    bool isRawPressed() const;
};