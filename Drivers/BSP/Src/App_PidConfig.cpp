#include "App_PidConfig.h"
#include "PidStorage.hpp"
#include "W25Q64.hpp"
#include "LineFollower_Interface.h"
#include "main.h"
#include "spi.h"
#include <cstdio>  // for sscanf, RTT_Log
#include <cstring> // for strncmp

#include "SEGGER_RTT.h"
#include "UartRingBuffer.hpp"

// 1. 定义全局 Buffer，并指定放到 RAM 域
// 如果没有特殊段，也可以去掉 section 属性，只用 aligned
#define SERIAL_BUF_SIZE 512
uint8_t dma_rx_buffer[SERIAL_BUF_SIZE] __attribute__((section(".RAM"), aligned(32)));

// 2. 实例化对象，传入全局 Buffer
extern UART_HandleTypeDef huart3;
UartRingBuffer serialRx(&huart3, dma_rx_buffer, SERIAL_BUF_SIZE);

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

    // === 加载并注入 Turn PID (ID 0) ===
    PidConfig cfgTurn = pidStore.get(PID_ID_TURN);
    LineFollower_SetPID(PID_ID_TURN, cfgTurn.kp, cfgTurn.ki, cfgTurn.kd);

    // === 【新增】加载并注入 Forward PID (ID 1) ===
    PidConfig cfgFwd = pidStore.get(PID_ID_FORWARD);
    LineFollower_SetPID(PID_ID_FORWARD, cfgFwd.kp, cfgFwd.ki, cfgFwd.kd);
    
    RTT_Log("[System] Applied PID: Turn P=%.3f I=%.3f D=%.3f | Forward P=%.3f I=%.3f D=%.3f\r\n",
            cfgTurn.kp, cfgTurn.ki, cfgTurn.kd,
            cfgFwd.kp, cfgFwd.ki, cfgFwd.kd);
}

// 【新增】串口初始化
void App_Serial_Init(void) {
    serialRx.init(); // 启动 DMA
    printf("[System] Serial RingBuffer Started.\r\n");
}

// 【新增】串口轮询任务
void App_Serial_Loop(void) {
    std::string cmd;

    // process() 是非阻塞的，如果没有完整的一行，它会立即返回 false
    // 如果返回 true，说明 cmd 里存了一行指令 (例如 "&LPID...#")
    if (serialRx.process(cmd)) {

        // 打印原始数据调试 (可选)
        // printf("[Rx] %s\r\n", cmd.c_str());

        // 调用之前的解析函数
        // .c_str() 返回 const char*，我们需要强转一下兼容接口
        App_Pid_Process_Command((char*)cmd.c_str());
    }
}

void App_Pid_Set_Temp(uint8_t id, float kp, float ki, float kd) {
    // 1. 更新 RAM 缓存
    pidStore.set(id, kp, ki, kd);

    // 2. 立即应用到电机控制器
    LineFollower_SetPID(id, kp, ki, kd);
    
    RTT_Log("[Tuning] Temp PID Set: P=%.3f I=%.3f D=%.3f (RAM only)\r\n", kp, ki, kd);
}

void App_Pid_Save(void) {
    // 写入 Flash
    // 注意：这会阻塞 CPU 约 50ms
    pidStore.save();
    RTT_Log("[System] PID Parameters Saved to Flash!\r\n");
}

// === 串口命令解析 ===
// 映射关系：LPID 对应 ID 0 (转向), FPID 对应 ID 1 (前进/速度)
// 你可以根据需要添加更多
int Get_Pid_ID_From_Name(char* name) {
    if (strcmp(name, "LPID") == 0) return PID_ID_TURN;     // ID 0
    if (strcmp(name, "FPID") == 0) return PID_ID_FORWARD;  // ID 1 (如果有)
    return -1; // 未知名称
}

void App_Pid_Process_Command(char* cmd_buffer) {
    // cmd_buffer 内容示例: "&LPID.P=1.5,I=0.2,D=0.5#"
    // 注意：环形缓冲区已经帮我们去掉了末尾的 \r 或 \n
    
    // 1. 简单校验帧头帧尾
    size_t len = strlen(cmd_buffer);
    if (cmd_buffer[0] != '&' || cmd_buffer[len - 1] != '#') {
        // 如果不是 & 开头，# 结尾，尝试处理 SAVE 指令
        if (strncmp(cmd_buffer, "SAVE", 4) == 0) {
            App_Pid_Save();
        } else {
            // 格式错误或垃圾数据，直接忽略
            // RTT_Log("[Error] Invalid Frame: %s\n", cmd_buffer);
        }
        return;
    }

    // 2. 解析数据
    char name[10];
    float p, i, d;
    
    // 使用 sscanf 进行格式化拆解
    // 格式解释:
    // &       : 跳过开头的 &
    // %[^.]   : 读取字符串直到遇到 '.'，存入 name (即读取 "LPID")
    // .P=     : 跳过 ".P="
    // %f      : 读取浮点数 p
    // ,I=     : 跳过 ",I="
    // %f      : 读取浮点数 i
    // ,D=     : 跳过 ",D="
    // %f      : 读取浮点数 d
    // #       : 匹配结尾的 #
    int args = sscanf(cmd_buffer, "&%[^.].P=%f,I=%f,D=%f#", name, &p, &i, &d);

    // 3. 校验解析结果 (必须成功读取到 name, p, i, d 这4个值)
    if (args == 4) {
        int id = Get_Pid_ID_From_Name(name);
        
        if (id >= 0) {
            // 执行参数设置
            App_Pid_Set_Temp(static_cast<uint8_t>(id), p, i, d);
            // 打印调试信息，确认收到的值
            RTT_Log("[Ack] Set %s (ID %d): P=%f, I=%f, D=%f\r\n", name, id, p, i, d);
        } else {
            RTT_Log("[Error] Unknown PID Name: %s\r\n", name);
        }
    } else {
        RTT_Log("[Error] Parse Failed! Check format.\r\n");
    }
}