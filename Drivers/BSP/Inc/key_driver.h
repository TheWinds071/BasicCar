#ifndef __KEY_DRIVER_H
#define __KEY_DRIVER_H

#include "main.h" // 包含HAL库定义

// 配置参数
#define DEBOUNCE_TICKS    20    // 消抖时间 (ms)
#define LONG_PRESS_TICKS  1000  // 长按判定时间 (ms)

// 按键状态枚举
typedef enum {
    KEY_STATE_IDLE = 0,    // 空闲
    KEY_STATE_DEBOUNCE,    // 消抖中
    KEY_STATE_PRESSED,     // 已按下
    KEY_STATE_LONG_PRESS   // 长按保持中
} KeyState_t;

// 按键事件结果（返回给主循环）
typedef enum {
    KEY_EVENT_NONE = 0,    // 无事件
    KEY_EVENT_CLICK,       // 单击
    KEY_EVENT_LONG_PRESS,  // 长按
    KEY_EVENT_RELEASE      // 松开 (可选)
} KeyEvent_t;

// 按键对象结构体
typedef struct {
    GPIO_TypeDef* port;     // GPIO端口
    uint16_t pin;           // GPIO引脚
    GPIO_PinState active_level; // 有效电平 (0:低电平有效/按下接地, 1:高电平有效)
    
    KeyState_t state;       // 当前状态机状态
    uint32_t start_tick;    // 状态开始的时间戳
} Key_t;

// 函数声明
void Key_Init(Key_t* key, GPIO_TypeDef* port, uint16_t pin, GPIO_PinState active_level);
KeyEvent_t Key_Scan(Key_t* key);

#endif