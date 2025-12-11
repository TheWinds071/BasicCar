#include "App_PidConfig.h"
#include "PidStorage.hpp"
#include "W25Q64.hpp"
#include "LineFollower_Interface.h"
#include "main.h"
#include "spi.h"
#include <cstdio>  // for sscanf, RTT_Log
#include <cstring> // for strncmp

#include "SEGGER_RTT.h"

// === 硬件对象实例化 ===
W25Q64 w25q(&hspi2, SPI2_CS_GPIO_Port, SPI2_CS_Pin);
PidStorage pidStore(w25q);

void App_Pid_Init(void) {
    // 1. 初始化 Flash 并加载参数
    bool valid = pidStore.load();
    
    if (valid) {
        RTT_Log("[System] PID loaded from Flash.\r\n");
    } else {
        RTT_Log("[System] Flash empty. Using defaults & Saving...\r\n");
        pidStore.save(); // 保存默认值
    }

    // 2. 获取参数
    PidConfig cfg = pidStore.get(PID_ID_TURN);

    // 3. 注入到 PID 对象 (通过你已有的接口)
    LineFollower_SetPID(cfg.kp, cfg.ki, cfg.kd);
    
    RTT_Log("[System] Applied PID: P=%.3f, I=%.3f, D=%.3f\r\n", cfg.kp, cfg.ki, cfg.kd);
}

void App_Pid_Set_Temp(uint8_t id, float kp, float ki, float kd) {
    // 1. 更新 RAM 缓存
    pidStore.set(id, kp, ki, kd);

    // 2. 立即应用到电机 (热更新)
    if (id == PID_ID_TURN) {
        LineFollower_SetPID(kp, ki, kd);
    }
    
    RTT_Log("[Tuning] Temp PID Set: P=%.3f D=%.3f (RAM only)\r\n", kp, kd);
}

void App_Pid_Save(void) {
    // 写入 Flash
    // 注意：这会阻塞 CPU 约 50ms
    pidStore.save();
    RTT_Log("[System] PID Parameters Saved to Flash!\r\n");
}

// === 串口命令解析 ===
// 支持指令:
// 1. "PID <kp> <ki> <kd>"  -> 修改参数
// 2. "SAVE"                -> 保存参数
void App_Pid_Process_Command(char* cmd_buffer) {
    float p, i, d;

    // 检查是否是 PID 设置指令
    if (strncmp(cmd_buffer, "PID", 3) == 0) {
        // 解析参数
        // 格式示例: "PID 0.15 0.0 0.8"
        int args = sscanf(cmd_buffer, "PID %f %f %f", &p, &i, &d);
        
        if (args == 3) {
            App_Pid_Set_Temp(PID_ID_TURN, p, i, d);
        } else {
            RTT_Log("[Error] Format: PID <Kp> <Ki> <Kd>\r\n");
        }
    }
    // 检查是否是保存指令
    else if (strncmp(cmd_buffer, "SAVE", 4) == 0) {
        App_Pid_Save();
    }
    else {
        RTT_Log("[Unkown] Cmd: %s\r\n", cmd_buffer);
    }
}