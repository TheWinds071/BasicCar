#pragma once
#include "W25Q64.hpp" // 引用你的底层 Flash 驱动
#include <cstring>

#include "SEGGER_RTT.h"

// 定义 ID: 目前寻线只需要一个转向 PID
#define PID_ID_TURN  0
#define PID_ID_FORWARD 1     // 预留前进 PID
#define MAX_PID_NUM  4       // 预留 4 组
#define PID_MAGIC    0x5AA5  // 校验魔数

struct PidConfig {
    float kp;
    float ki;
    float kd;
};

struct FlashLayout {
    uint32_t magic;
    PidConfig pids[MAX_PID_NUM];
};

class PidStorage {
private:
    W25Q64& _flash;
    FlashLayout _cache{};
    uint32_t _addr;

public:
    PidStorage(W25Q64& f, uint32_t addr = 0x7FF000) : _flash(f), _addr(addr) {
        std::memset(&_cache, 0, sizeof(_cache));
    }

    // 上电初始化：读 Flash 到 RAM
    bool load() {
        if (!_flash.init()) return false;

        _flash.readData(_addr, (uint8_t*)&_cache, sizeof(_cache));

        if (_cache.magic == PID_MAGIC) {
            return true;
        } else {
            // Flash 为空，加载默认值
            _cache.magic = PID_MAGIC;
            _cache.pids[PID_ID_TURN] = {0.1f, 0.0f, 0.2f}; // 默认参数
            return false;
        }
    }

    // 保存 RAM 到 Flash (耗时!)
    void save() {
        _cache.magic = PID_MAGIC; 
        
        // 1. 擦除
        _flash.eraseSector(_addr);
        
        // 2. 写入
        _flash.writePage(_addr, (uint8_t*)&_cache, sizeof(_cache));

        // === 【新增调试代码】立即回读检查 ===
        FlashLayout tempCheck{};
        _flash.readData(_addr, (uint8_t*)&tempCheck, sizeof(tempCheck));
        
        if (tempCheck.magic == PID_MAGIC) {
            RTT_Log("[Debug] Verify Success: Magic is 0x%X\r\n", tempCheck.magic);
        } else {
            RTT_Log("[Debug] Verify FAILED! Wrote 0x%X, Read 0x%X\r\n", PID_MAGIC, tempCheck.magic);
        }
    }

    // 读写 RAM 缓存
    void set(uint8_t id, float p, float i, float d) {
        if (id < MAX_PID_NUM) _cache.pids[id] = {p, i, d};
    }

    PidConfig get(uint8_t id) {
        if (id < MAX_PID_NUM) return _cache.pids[id];
        return {0,0,0};
    }
};