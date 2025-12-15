#include "LineFollower.h"
#include "tim.h"
#include "ui.h"

// === 对象实例化 ===
// 假设：
// 使用 TIM1
// 左电机连接 TIM1 CH1 和 CH2

static LineFollower* controller = nullptr;

// C 接口：初始化
void LineFollower_Init(void) {
    // 假设电机接在 TIM1
    // 左电机: CH1, CH2
    // 右电机: CH3, CH4
    static LineFollower static_instance(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4);
    controller = &static_instance;

    // 设置基础速度 (0.0 - 1.0)
    // 例如 0.4 代表 40% 的占空比
    // controller->setBaseSpeed(0.1f);

    controller->begin();
}

// C 接口：中断调用
void LineFollower_OnTimer(void) {
    if (controller != nullptr) {
        uint8_t conformedQustion = UI_GetConfirmedQuestion();
        controller->updateISR(conformedQustion);
    }
}

// 动态调整 PID 接口
void LineFollower_SetPID(uint8_t id,float kp, float ki, float kd) {
    if (controller != nullptr) {
        controller->tunePid(id, kp, ki, kd);
    }
}

// 动态调整速度接口
void LineFollower_SetSpeed(float speed) {
    if (controller != nullptr) {
        controller->setBaseSpeed(speed);
    }
}

// 重置航向参考接口
void LineFollower_SetYaw() {
    if (controller != nullptr) {
        controller->resetYawRef();
    }
}

// 设置航向参考接口
void LineFollower_SetYawRef(float yaw_deg) {
    if (controller != nullptr) {
        controller->setYawRefDeg(yaw_deg);
    }
}