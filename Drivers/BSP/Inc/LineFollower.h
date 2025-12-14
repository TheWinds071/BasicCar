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

// === 新增：引用 main.c 中的 IMU 姿态角数组 ===
// 你在 main.c 里用的是 User_YPR（注意大小写），这里用 extern 引用即可。
// User_YPR[0] = yaw, [1] = pitch, [2] = roll（按常见 IMU 输出约定）
extern float User_YPR[3];

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

    // === 新增：直线航向保持相关 ===
    bool  _yaw_ref_inited = false;
    float _yaw_ref_deg    = 0.0f; // 直线目标航向（单位：度；与你 IMU 输出一致）

    // 将角度误差归一化到 [-180, 180]，避免 yaw 跳变导致 PID 爆炸
    static float wrapAngleDeg(float err_deg) {
        while (err_deg > 180.0f) err_deg -= 360.0f;
        while (err_deg < -180.0f) err_deg += 360.0f;
        return err_deg;
    }

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
          _pidForward(1.0f, 0.0f, 0.0f, -1.0f, 1.0f),    // 初始化直行 PID (输出限幅 -1.0~1.0)
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

    // 外部强制重新锁定直线航向
    void resetYawRef() {
        _yaw_ref_inited = false;
        _pidForward.reset();
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

        // if (raw == 0) {
        //     // 脱轨处理：这里简单刹车，也可以写逻辑让它原地转圈找线
        //     setSingleMotor(_ch_L1, _ch_L2, 0.0f);
        //     setSingleMotor(_ch_R1, _ch_R2, 0.0f);
        //     // 可选：重置 PID 积分项，防止回到线上时突然猛冲
        //     _pidTurn.reset();
        //     return;
        // }

        float sum = 0;
        int count = 0;
        if (raw & GPIO_PIN_15) { sum += -5.0f; count++; }
        // if (raw & GPIO_PIN_14) { sum += -2.0f; count++; }
        // if (raw & GPIO_PIN_13) { sum += -1.0f; count++; }
        if (raw & GPIO_PIN_12) { sum +=  2.0f; count++; }
        if (raw & GPIO_PIN_11) { sum +=  0.0f; count++; }
        if (raw & GPIO_PIN_10) { sum +=  2.0f; count++; }
        if (raw & GPIO_PIN_8)  { sum +=  5.0f; count++; }

        position_error = sum / count;

        // 3. 使用 PID 模板计算转向值
        // 目标是 0.0 (中心)，当前是 position_error
        float turn_adjust = _pidTurn.compute(0.0f, position_error);

        // 4. === 新增：FPID（走直线/航向保持） ===
        // 4.1 锁定“直线目标航向”
        float yaw_now = User_YPR[0]; // yaw（单位按你的 IMU 输出）
        if (!_yaw_ref_inited) {
            _yaw_ref_deg = yaw_now;
            _yaw_ref_inited = true;
        }

        // 4.2 计算航向误差并归一化
        float yaw_err = wrapAngleDeg(_yaw_ref_deg - yaw_now);

        // 4.3 FPID 输出（把误差当 measured，目标设为 0）
        // yaw_adjust > 0 表示需要往一个方向修正（具体左右取决于你的电机正反定义）
        float yaw_adjust = _pidForward.compute(0.0f, yaw_err);

        // 5. === 混合运动计算 ===
        // 总差速修正 = 寻线转向修正 + 航向修正
        // float diff = turn_adjust + yaw_adjust;
        float diff = -yaw_adjust;

        RTT_Log("YawErr: %.2f, YawAdj: %.2f, Diff: %.2f\r\n", yaw_err, yaw_adjust, diff);

        float speed_l = _base_speed - diff;
        float speed_r = _base_speed + diff;

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
