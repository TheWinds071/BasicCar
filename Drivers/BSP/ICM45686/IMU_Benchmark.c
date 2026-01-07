#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdio.h>

#include "SEGGER_RTT.h"

/* 你的函数原型（按实际头文件改/删） */
void IMU_AHRSupdate(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float mx, float my, float mz);

/* 只需调用一次：开启 DWT 周期计数器 */
void DWT_CycleCounterInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable trace
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Enable cycle counter
}

/* 返回：IMU_AHRSupdate 的平均耗时（微秒） */
float IMU_Benchmark_AHRSupdate_AvgUs(uint32_t loops)
{
    /* 480MHz */
    const float cycles_per_us = 480.0f;

    /* 给一组固定输入，避免编译器/分支抖动（你也可以换成真实采样值） */
    const float gx = 0.01f, gy = -0.02f, gz = 0.005f;
    const float ax = 0.0f,  ay = 0.0f,   az = 1.0f;
    const float mx = 0.2f,  my = 0.0f,   mz = 0.5f;

    if (loops == 0) loops = 1;

    /* 确保 DWT 已开启 */
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0)
        DWT_CycleCounterInit();

    /* 预热：避免首次调用带来的 cache/分支预测影响 */
    for (uint32_t i = 0; i < 50; i++)
        IMU_AHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);

    /* 正式统计 */
    uint64_t sum_cycles = 0;

    for (uint32_t i = 0; i < loops; i++)
    {
        uint32_t t0 = DWT->CYCCNT;
        IMU_AHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);
        uint32_t dt = DWT->CYCCNT - t0;
        sum_cycles += (uint64_t)dt;
    }

    float avg_cycles = (float)sum_cycles / (float)loops;
    float avg_us = avg_cycles / cycles_per_us;

    return avg_us;
}

/* 示例：在 main 里调用 */
void IMU_PrintAhrsAvgTime(void)
{
    float avg_us = IMU_Benchmark_AHRSupdate_AvgUs(1000);
    RTT_Log("IMU_AHRSupdate avg: %.3f us (480MHz, loops=1000)\r\n", avg_us);
}
