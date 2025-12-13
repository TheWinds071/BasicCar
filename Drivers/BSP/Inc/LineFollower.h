#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include "main.h"
#include "Pid.hpp" // 包含刚才定义的模板
#include <cmath>

#include "PidStorage.hpp"

// ================== 配置参数 ==================
#define LF_SENSOR_MASK   0x9D00  // PA15,PA12,PA11,PA10,PA8
//  APB2=240M, 20kHz -> ARR=11999
#define LF_PWM_PERIOD    11999

class LineFollower {
private:
    TIM_HandleTypeDef* _htim;
    uint32_t _ch_L1, _ch_L2; // 左电机通道
    uint32_t _ch_R1, _ch_R2; // 右电机通道
    uint32_t _pwm_arr;

    // === 核心变化：引入 PID 对象 ===
    PidController<float> _pidTurn;
    PidController<float> _pidForward;

    float _base_speed; // 基础速度 (0.0 - 1.0)

    // 内部函数：设置单边电机 (DRV8870 慢衰减)
    void setSingleMotor(uint32_t ch1, uint32_t ch2, float speed) {
        if (speed > 1.0f) speed = 1.0f;
        if (speed < -1.0f) speed = -1.0f;

        uint32_t duty_inv;
        if (speed >= 0.0f) {
            duty_inv = (uint32_t)((1.0f - speed) * _pwm_arr);
            __HAL_TIM_SET_COMPARE(_htim, ch1, _pwm_arr);
            __HAL_TIM_SET_COMPARE(_htim, ch2, duty_inv);
        } else {
            float abs_s = -speed;
            duty_inv = (uint32_t)((1.0f - abs_s) * _pwm_arr);
            __HAL_TIM_SET_COMPARE(_htim, ch1, duty_inv);
            __HAL_TIM_SET_COMPARE(_htim, ch2, _pwm_arr);
        }
    }

public:

    /**
     * @brief 构造函数
     * @param kp, ki, kd PID 参数
     */
    LineFollower(TIM_HandleTypeDef* htim, uint32_t l1, uint32_t l2, uint32_t r1, uint32_t r2)
        : _htim(htim), _ch_L1(l1), _ch_L2(l2), _ch_R1(r1), _ch_R2(r2),
          _pidTurn(0.1f, 0.0f, 0.2f, -1.0f, 1.0f),      // 初始化转向 PID
          _pidForward(1.0f, 0.0f, 0.0f, 0.0f, 1.0f),    // 【新增】初始化前进 PID (输出限幅 0~1.0)
          _base_speed(0.0f)
    {
        _pwm_arr = 0;
    }

    void begin() {
        _pwm_arr = __HAL_TIM_GET_AUTORELOAD(_htim);

        HAL_TIM_PWM_Start(_htim, _ch_L1);
        HAL_TIM_PWM_Start(_htim, _ch_L2);
        HAL_TIM_PWM_Start(_htim, _ch_R1);
        HAL_TIM_PWM_Start(_htim, _ch_R2);

    }

    // 设置基础速度
    void setBaseSpeed(float speed) {
        _base_speed = speed;
    }

    // === 修改：支持指定 ID 调参 ===
    void tunePid(uint8_t id, float kp, float ki, float kd) {
        if (id == PID_ID_TURN) {
            _pidTurn.setTunings(kp, ki, kd);
        }
        else if (id == PID_ID_FORWARD) {
            _pidForward.setTunings(kp, ki, kd);
        }
    }

    // === 中断调用核心 ===
    void updateISR() {
        // 1. 读取传感器 (IDR & Mask)
        // 假设传感器：遇黑线为 1
        uint16_t raw = (GPIOA->IDR) & LF_SENSOR_MASK;

        // 2. 计算当前位置偏差 (Measured)
        float position_error = 0.0f;

        if (raw == 0) {
            // 脱轨处理：这里简单刹车，也可以写逻辑让它原地转圈找线
            setSingleMotor(_ch_L1, _ch_L2, 0.0f);
            setSingleMotor(_ch_R1, _ch_R2, 0.0f);
            // 可选：重置 PID 积分项，防止回到线上时突然猛冲
            _pidTurn.reset();
            return;
        }

        float sum = 0;
        int count = 0;
        if (raw & GPIO_PIN_15) { sum += -3.0f; count++; }
        // if (raw & GPIO_PIN_14) { sum += -2.0f; count++; }
        // if (raw & GPIO_PIN_13) { sum += -1.0f; count++; }
        if (raw & GPIO_PIN_12) { sum +=  0.0f; count++; }
        if (raw & GPIO_PIN_11) { sum +=  1.0f; count++; }
        if (raw & GPIO_PIN_10) { sum +=  2.0f; count++; }
        if (raw & GPIO_PIN_8)  { sum +=  3.0f; count++; }

        position_error = sum / count;

        // 3. 使用 PID 模板计算转向值
        // 目标是 0.0 (中心)，当前是 position_error
        float turn_adjust = _pidTurn.compute(0.0f, position_error);

        // 4. 混合速度
        // 如果 turn_adjust > 0，说明偏右(error>0)，需要左转(左轮减速，右轮加速)
        // 注意：根据你的电机实际接线，可能需要调换 +/- 号
        float speed_l = _base_speed - turn_adjust;
        float speed_r = _base_speed + turn_adjust;

        setSingleMotor(_ch_L1, _ch_L2, speed_l);
        setSingleMotor(_ch_R2, _ch_R1, speed_r);
    }
};

#ifdef __cplusplus
extern "C" {
#endif

    // 初始化寻线功能 (配置电机、PID参数)
    void LineFollower_Init(void);

    // 核心控制循环 (必须在 1ms 定时器中断中调用)
    void LineFollower_OnTimer(void);

    // 可选：设置基础速度 (0.0 ~ 1.0)
    void LineFollower_SetSpeed(float speed);

    void LineFollower_SetPID(float kp, float ki, float kd);

#ifdef __cplusplus
}
#endif

#endif
