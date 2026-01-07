#pragma once
#include "main.h"
#include <cstdarg>
#include <cstdio>
#include <cstring>

class UartTxRingBuffer {
public:
    UartTxRingBuffer(UART_HandleTypeDef* huart, uint8_t* buffer, uint16_t size)
        : _huart(huart), _buf(buffer), _size(size) {}

    void init() {
        _head = 0;
        _tail = 0;
        _dma_busy = false;
    }

    // 写入发送队列；队列满则尽量写入，剩余丢弃并返回 false
    bool write(const uint8_t* data, uint16_t len) {
        if (!data || len == 0) return false;

        bool ok = true;
        for (uint16_t i = 0; i < len; i++) {
            uint16_t next = (uint16_t)((_head + 1U) % _size);
            if (next == _tail) { ok = false; break; } // full
            _buf[_head] = data[i];
            _head = next;
        }
        kickTx();
        return ok;
    }

    bool writeString(const char* s) {
        if (!s) return false;
        return write(reinterpret_cast<const uint8_t*>(s), (uint16_t)strlen(s));
    }

    bool printf(const char* fmt, ...) {
        char tmp[128];
        va_list ap;
        va_start(ap, fmt);
        int n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
        va_end(ap);

        if (n <= 0) return false;
        uint16_t len = (n >= (int)sizeof(tmp)) ? (uint16_t)(sizeof(tmp) - 1) : (uint16_t)n;
        return write(reinterpret_cast<const uint8_t*>(tmp), len);
    }

    // 在 HAL_UART_TxCpltCallback 里调用
    void onTxCplt(UART_HandleTypeDef* huart) {
        if (huart != _huart) return;
        _dma_busy = false;
        kickTx();
    }

private:
    UART_HandleTypeDef* _huart = nullptr;

    uint8_t* _buf = nullptr;
    uint16_t _size = 0;

    volatile uint16_t _head = 0;
    volatile uint16_t _tail = 0;
    volatile bool _dma_busy = false;

    static constexpr uint16_t CHUNK_MAX = 64; // 每次DMA最多发64字节，避免长时间占用

    void kickTx() {
        if (_huart == nullptr) return;
        if (_dma_busy) return;
        if (_head == _tail) return; // empty

        // tail->head 之间的连续长度
        uint16_t len;
        if (_head > _tail) len = (uint16_t)(_head - _tail);
        else               len = (uint16_t)(_size - _tail);

        if (len > CHUNK_MAX) len = CHUNK_MAX;

        _dma_busy = true;

        if (HAL_UART_Transmit_DMA(_huart, &_buf[_tail], len) != HAL_OK) {
            _dma_busy = false;
            return;
        }

        // 先推进 tail（这段已经交给DMA发送）
        _tail = (uint16_t)((_tail + len) % _size);
    }
};