#pragma once

#include <cstdint>

class Prompt {
public:
    // duration_ms: 提示持续时间，比如 80~200ms
    static void init();
    static void once(uint32_t duration_ms);
    static void tick(uint32_t now_ms);

private:
    static void on();
    static void off();

private:
    static inline bool s_active = false;
    static inline uint32_t s_offTime = 0;
};