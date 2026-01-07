#include "DbgSerial.hpp"
#include "usart.h"   // 需要这里面有 huart3

uint8_t g_tx3_buf[512] __attribute__((section(".RAM"), aligned(32)));;

UartTxRingBuffer g_dbgTx3(&huart3, g_tx3_buf, sizeof(g_tx3_buf));