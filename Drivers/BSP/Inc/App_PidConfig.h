#ifndef APP_PID_CONFIG_H
#define APP_PID_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

    // 1. 系统初始化调用：Flash初始化 -> 读取 -> 注入控制器
    void App_Pid_Init(void);

    // 2. 在线修改 PID (只改 RAM 并立即生效，不写 Flash)
    // id: 0 (PID_ID_TURN)
    void App_Pid_Set_Temp(uint8_t id, float kp, float ki, float kd);

    // 3. 保存当前参数到 Flash
    void App_Pid_Save(void);

    // 4. 串口命令解析器
    // 传入接收到的字符串，例如 "PID 0.5 0.0 1.2" 或 "SAVE"
    void App_Pid_Process_Command(char* cmd_buffer);

#ifdef __cplusplus
}
#endif

#endif