/* Core/Src/app_entry.cpp */
#include "app_entry.h"
#include "main.h"       // 包含 HAL 库
#include "button.hpp"   // 包含上一段我们写的 Button 类
#include "SEGGER_RTT.h"
#include "App_PidConfig.h"
#include "LineFollower_Interface.h"
#include "Prompt.hpp"
#include "u8g2.h"
#include "ui.h"

extern u8g2_t u8g2;
extern float User_YPR[];

static float g_yaw_mark = 0.0f;

// --- C++ 对象实例化 (全局) ---
// 此时已经在 C++ 环境中了，对象构造函数会被正确调用
Button btn(USER_KEY_GPIO_Port, USER_KEY_Pin, true);

// 定义回调函数
void onOkClick() {
    RTT_Log("Button Clicked!\r\n");
    App_Pid_Set_Temp(0,2.0f,3.0f,4.0f);
    App_Pid_Save();
}

void setYawRef() {
    // 1) 记录长按时刻的 yaw
    g_yaw_mark = User_YPR[0];
    LineFollower_SetYawRef(g_yaw_mark);
    LineFollower_SetYaw();
}

// --- 引导函数实现 ---
// 这个函数接管了 main.c 的控制权
void App_Start(void) {

    // 1. C++ 逻辑初始化
    btn.attachClick(onOkClick);
    btn.attachLongPress(setYawRef); // 长按重置航向参考
    // 初始化三键 UI
    UI_Init();
    //初始化声光提示
    Prompt::init();
    Prompt::once(120); // 开机提示一下（非阻塞）
    // 2. C++ 主循环 (替代 main.c 的 while(1))
    uint32_t oled_update_counter = 0;
    while (1) {

        // 扫描按键
        btn.scan();
        // 扫描三键（PE6/PE4/PE5）并更新 UI 状态
        UI_Button_Update();

        App_Serial_Loop();

        // 绘制 UI
        UI_Render();

        Prompt::tick(HAL_GetTick());

    }
}