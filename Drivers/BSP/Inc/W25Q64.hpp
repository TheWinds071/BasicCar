#ifndef W25Q64_HPP
#define W25Q64_HPP

#include "main.h"
#include <cstring>

#include "SEGGER_RTT.h"

// W25Q64 指令集
#define W25Q_CMD_WRITE_ENABLE      0x06
#define W25Q_CMD_READ_STATUS_REG1  0x05
#define W25Q_CMD_READ_DATA         0x03
#define W25Q_CMD_PAGE_PROGRAM      0x02
#define W25Q_CMD_SECTOR_ERASE_4KB  0x20
#define W25Q_CMD_JEDEC_ID          0x9F

// 存储参数的扇区地址 (W25Q64 总大小 8MB)
// 使用最后一个 4KB 扇区: 0x7FF000
#define PID_STORAGE_ADDRESS        0x7FF000
#define PID_MAGIC_NUMBER           0xDEADBEEF // 用于校验数据有效性

// PID 数据结构体
struct FlashPidData {
    uint32_t magic; // 校验位
    float kp;
    float ki;
    float kd;
    // 可以继续添加其他参数，如目标速度等
};

class W25Q64 {
private:
    SPI_HandleTypeDef* _hspi;
    GPIO_TypeDef* _cs_port;
    uint16_t _cs_pin;

    // 底层 CS 控制
    void csLow() {
        HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
    }

    void csHigh() {
        HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
    }

    // SPI 收发封装
    uint8_t spiSwap(uint8_t data) {
        uint8_t rx_data = 0;
        HAL_SPI_TransmitReceive(_hspi, &data, &rx_data, 1, 100);
        return rx_data;
    }

    // 等待芯片忙碌状态结束
    void waitForReady() {
        uint8_t status;
        do {
            csLow();
            spiSwap(W25Q_CMD_READ_STATUS_REG1);
            status = spiSwap(0xFF); // Dummy byte to clock out status
            csHigh();
        } while ((status & 0x01) == 0x01); // 检查 BUSY 位 (Bit 0)
    }

    // 写使能
    void writeEnable() {
        csLow();
        spiSwap(W25Q_CMD_WRITE_ENABLE);
        csHigh();
    }

public:
    W25Q64(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin)
        : _hspi(hspi), _cs_port(cs_port), _cs_pin(cs_pin) {}

    // 初始化 (简单的 ID 读取测试)
    bool init() {
        csHigh();
        HAL_Delay(20);  // 增加延迟到20ms

        // 重试3次
        for(int retry = 0; retry < 3; retry++) {
            csLow();
            spiSwap(W25Q_CMD_JEDEC_ID);
            uint8_t mid = spiSwap(0xFF);
            uint8_t mtype = spiSwap(0xFF);
            uint8_t mcap = spiSwap(0xFF);
            csHigh();

            RTT_Log("[W25Q64] Init Attempt %d: MID=0x%02X, Type=0x%02X, Cap=0x%02X\r\n",
                    retry+1, mid, mtype, mcap);

            if (mid == 0xEF) {
                return true;
            }

            HAL_Delay(10);
        }

        RTT_Log("[W25Q64] Init FAILED after 3 retries!\r\n");
        return false;
    }

    // 擦除一个 4KB 扇区
    void eraseSector(uint32_t address) {
        waitForReady();
        writeEnable();

        csLow();
        spiSwap(W25Q_CMD_SECTOR_ERASE_4KB);
        spiSwap((address >> 16) & 0xFF);
        spiSwap((address >> 8) & 0xFF);
        spiSwap(address & 0xFF);
        csHigh();

        waitForReady(); // 擦除需要时间 (典型值 45ms)
    }

    // 写入数据 (页编程)
    void writePage(uint32_t address, uint8_t* data, uint16_t size) {
        waitForReady();
        writeEnable();

        csLow();
        spiSwap(W25Q_CMD_PAGE_PROGRAM);
        spiSwap((address >> 16) & 0xFF);
        spiSwap((address >> 8) & 0xFF);
        spiSwap(address & 0xFF);

        for (uint16_t i = 0; i < size; i++) {
            spiSwap(data[i]);
        }
        csHigh();

        waitForReady();
    }

    // 读取数据
    void readData(uint32_t address, uint8_t* buffer, uint16_t size) {
        waitForReady();

        csLow();
        spiSwap(W25Q_CMD_READ_DATA);
        spiSwap((address >> 16) & 0xFF);
        spiSwap((address >> 8) & 0xFF);
        spiSwap(address & 0xFF);

        for (uint16_t i = 0; i < size; i++) {
            buffer[i] = spiSwap(0xFF);
        }
        csHigh();
    }

    // === 专门用于 PID 的封装函数 ===

    // 保存 PID 参数到 Flash
    void savePidParams(float kp, float ki, float kd) {
        FlashPidData data{};
        data.magic = PID_MAGIC_NUMBER;
        data.kp = kp;
        data.ki = ki;
        data.kd = kd;

        // 1. 擦除扇区 (Flash 必须先擦后写)
        eraseSector(PID_STORAGE_ADDRESS);

        // 2. 写入结构体
        // 注意：结构体必须小于 256 字节（一页），否则需要分页逻辑
        // PID 参数很小，直接写没问题
        writePage(PID_STORAGE_ADDRESS, reinterpret_cast<uint8_t *>(&data), sizeof(FlashPidData));
    }

    // 从 Flash 读取 PID 参数
    // 如果 Flash 为空或无效，返回 false，并加载默认值
    bool loadPidParams(float& kp, float& ki, float& kd) {
        FlashPidData data{};
        readData(PID_STORAGE_ADDRESS, reinterpret_cast<uint8_t *>(&data), sizeof(FlashPidData));

        if (data.magic == PID_MAGIC_NUMBER) {
            kp = data.kp;
            ki = data.ki;
            kd = data.kd;
            return true; // 数据有效
        } else {
            // Flash 为空 (0xFF) 或数据损坏
            // 这里可以设置默认值
            kp = 0.0f;
            ki = 0.0f;
            kd = 0.0f;
            return false; // 需要重新校准或使用默认值
        }
    }
};

#endif // W25Q64_HPP