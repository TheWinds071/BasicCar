#pragma once
#include "main.h"
#include <string>

class UartRingBuffer {
private:
    UART_HandleTypeDef* _huart;
    uint8_t* _rx_buffer;      // 改成指针，指向外部内存
    uint16_t _buf_size;       // 记录大小
    uint16_t _tail;
    char _line_buffer[128]{};
    uint16_t _line_idx;

public:
    // 构造函数：传入 buffer 指针和大小
    UartRingBuffer(UART_HandleTypeDef* huart, uint8_t* buffer, uint16_t size)
        : _huart(huart), _rx_buffer(buffer), _buf_size(size), _tail(0), _line_idx(0) {}

    void init() {
        // 启动 DMA
        HAL_UART_Receive_DMA(_huart, _rx_buffer, _buf_size);
    }

    bool process(std::string& output_cmd) {
        // 计算 Head (注意用 _buf_size)
        uint16_t head = _buf_size - __HAL_DMA_GET_COUNTER(_huart->hdmarx);

        if (head == _tail) return false;

        bool cmd_found = false;
        while (_tail != head) {
            uint8_t byte = _rx_buffer[_tail];

            if (byte == '\n' || byte == '\r') {
                if (_line_idx > 0) {
                    _line_buffer[_line_idx] = '\0';
                    output_cmd = std::string(_line_buffer);
                    _line_idx = 0;
                    cmd_found = true;
                }
            } else {
                if (_line_idx < sizeof(_line_buffer) - 1) {
                    _line_buffer[_line_idx++] = (char)byte;
                }
            }

            _tail++;
            if (_tail >= _buf_size) _tail = 0;
        }
        return cmd_found;
    }
};