/* Core/Src/app_entry.cpp */
#include "app_entry.h"
#include "main.h"       // 包含 HAL 库
#include "button.hpp"   // 包含上一段我们写的 Button 类
#include "SEGGER_RTT.h"
#include "App_PidConfig.h"

// --- C++ 对象实例化 (全局) ---
// 此时已经在 C++ 环境中了，对象构造函数会被正确调用
Button btn(USER_KEY_GPIO_Port, USER_KEY_Pin, true);

// 定义回调函数
void onOkClick() {
    RTT_Log("Button Clicked!\r\n");
    App_Pid_Set_Temp(0,2.0f,3.0f,4.0f);
    App_Pid_Save();
}

// --- 引导函数实现 ---
// 这个函数接管了 main.c 的控制权
void App_Start(void) {

    // 1. C++ 逻辑初始化
    btn.attachClick(onOkClick);


    // 2. C++ 主循环 (替代 main.c 的 while(1))
    while (1) {

        // 扫描按键
        btn.scan();

        App_Serial_Loop();
    }
}