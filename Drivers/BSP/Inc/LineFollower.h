#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include "main.h"
#include "Pid.hpp"
#include <cmath>

#include "PidStorage.hpp"
#include "Prompt.hpp"

// ================== 配置参数 ==================
#define LF_SENSOR_MASK   0x9D00  // PA15,PA12,PA11,PA10,PA8
#define LF_PWM_PERIOD    11999

extern float User_YPR[3];

class LineFollower {
private:
    TIM_HandleTypeDef* _htim;
    uint32_t _ch_L1, _ch_L2;
    uint32_t _ch_R1, _ch_R2;
    uint32_t _pwm_arr;

    PidController<float> _pidTurn;
    PidController<float> _pidForward;

    float _base_speed;

    bool  _yaw_ref_inited = false;
    float _yaw_ref_deg    = 0.0f;

    static float wrapAngleDeg(float err_deg) {
        while (err_deg > 180.0f) err_deg -= 360.0f;
        while (err_deg < -180.0f) err_deg += 360.0f;
        return err_deg;
    }

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

    // ====== 公共运动合成 ======
    void setEndSpeed(float turn_adjust, float yaw_adjust) {
        float diff = turn_adjust - yaw_adjust;
        float speed_l = _base_speed - diff;
        float speed_r = _base_speed + diff;

        setSingleMotor(_ch_L1, _ch_L2, speed_l);
        setSingleMotor(_ch_R2, _ch_R1, speed_r);
    }

    // ====== 传感器位置误差（带 count==0 保护）======
    bool calcPositionError(uint16_t raw, float& position_error_out) {
        float sum = 0.0f;
        int count = 0;

        if (raw & GPIO_PIN_15) { sum += -5.0f; count++; }
        if (raw & GPIO_PIN_12) { sum +=  2.0f; count++; }
        if (raw & GPIO_PIN_11) { sum +=  0.0f; count++; }
        if (raw & GPIO_PIN_10) { sum +=  2.0f; count++; }
        if (raw & GPIO_PIN_8)  { sum +=  5.0f; count++; }

        if (count == 0) return false;
        position_error_out = sum / (float)count;
        return true;
    }

    // ====== 直线段：航向保持（raw==0）======
    void driveStraightYawHold() {
        float yaw_now = User_YPR[0];
        if (!_yaw_ref_inited) {
            _yaw_ref_deg = yaw_now;
            _yaw_ref_inited = true;
        }

        float yaw_err = wrapAngleDeg(yaw_now - _yaw_ref_deg);
        float yaw_adjust = _pidForward.compute(0.0f, yaw_err);

        setEndSpeed(0.0f, yaw_adjust);
    }

    // ====== 半圆段：循线（raw!=0）======
    void driveArcLineFollow(uint16_t raw) {
        float position_error = 0.0f;
        if (!calcPositionError(raw, position_error)) {
            // 保险：算不出误差就停
            setSingleMotor(_ch_L1, _ch_L2, 0.0f);
            setSingleMotor(_ch_R1, _ch_R2, 0.0f);
            return;
        }

        float turn_adjust = _pidTurn.compute(0.0f, position_error);
        setEndSpeed(turn_adjust, 0.0f);
    }

    // ================== Q1 状态机 ==================
    enum class Q1State : uint8_t { Idle=0, GoStraight_AB, StopAtB };
    Q1State _q1_state = Q1State::Idle;
    bool _q1_prev_hasLine = false;
    bool _q1_prompted = false;

    void q1_enter(Q1State s) {
        _q1_state = s;
        _q1_prompted = false;
    }

    // ================== Q2 状态机：A->B->C->D->A ==================
    enum class Q2State : uint8_t { Idle=0, Straight_AB, Arc_BC, Straight_CD, Arc_DA, Done };
    Q2State _q2_state = Q2State::Idle;
    bool _q2_prev_hasLine = false;
    bool _q2_prompted = false;

    void q2_enter(Q2State s) {
        _q2_state = s;
        _q2_prompted = false;
    }

public:
    LineFollower(TIM_HandleTypeDef* htim, uint32_t l1, uint32_t l2, uint32_t r1, uint32_t r2)
        : _htim(htim), _ch_L1(l1), _ch_L2(l2), _ch_R1(r1), _ch_R2(r2),
          _pidTurn(0.1f, 0.0f, 0.2f, -1.0f, 1.0f),
          _pidForward(1.0f, 0.0f, 0.0f, -1.0f, 1.0f),
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

    void setBaseSpeed(float speed) { _base_speed = speed; }

    void resetYawRef() {
        _yaw_ref_inited = false;
        _pidForward.reset();
    }

    void setYawRefDeg(float yaw_deg) {
        _yaw_ref_deg = yaw_deg;
        _yaw_ref_inited = true;
        _pidForward.reset();
    }

    void tunePid(uint8_t id, float kp, float ki, float kd) {
        if (id == PID_ID_TURN) _pidTurn.setTunings(kp, ki, kd);
        else if (id == PID_ID_FORWARD) _pidForward.setTunings(kp, ki, kd);
    }

    // 题号变化时复位（推荐你在 OnTimer 用 lastQ 调用；即使不调用也能跑，但更稳）
    void onQuestionChanged(uint8_t q) {
        if (q != 1) { _q1_state = Q1State::Idle; _q1_prompted = false; }
        if (q != 2) { _q2_state = Q2State::Idle; _q2_prompted = false; }
    }

    // Q2 显式启动：进第二题时在 A 点提示一次
    void q2_start_from_A() {
        // 你保证起跑 raw==0，所以 prev_hasLine=false 合理
        _q2_prev_hasLine = false;
        resetYawRef();
        q2_enter(Q2State::Straight_AB);
        // A 点提示一次
        Prompt::once(120);
        _q2_prompted = true;
    }

    // ===== ISR 主逻辑 =====
    void updateISR(uint8_t conformedQuestion) {
        uint16_t raw = (GPIOA->IDR) & LF_SENSOR_MASK;
        bool hasLine = (raw != 0);

        switch (conformedQuestion) {
            case 1: { // Q1: A->B，到B停下并提示一次（状态机）
                bool rising = (!_q1_prev_hasLine) && hasLine; // 无线->有线 到B

                switch (_q1_state) {
                    case Q1State::Idle:
                        setBaseSpeed(0.10f);
                        resetYawRef();
                        q1_enter(Q1State::GoStraight_AB);
                        break;

                    case Q1State::GoStraight_AB:
                        setBaseSpeed(0.10f);
                        // 直线段：航向保持
                        driveStraightYawHold();
                        if (rising) {
                            q1_enter(Q1State::StopAtB);
                        }
                        break;

                    case Q1State::StopAtB:
                        setSingleMotor(_ch_L1, _ch_L2, 0.0f);
                        setSingleMotor(_ch_R1, _ch_R2, 0.0f);

                        if (!_q1_prompted) {
                            Prompt::once(120);
                            _q1_prompted = true;
                        }
                        break;
                }

                _q1_prev_hasLine = hasLine;
                break;
            }

            case 2: { // Q2: A->B->C->D->A，每过点提示一次（状态机）
                // 边沿
                bool rising  = (!_q2_prev_hasLine) && hasLine;      // 无线->有线：B 或 D
                bool falling = (_q2_prev_hasLine) && (!hasLine);    // 有线->无线：C 或 A

                switch (_q2_state) {
                    case Q2State::Idle:
                        // 兜底启动（如果你在外层已经调用 q2_start_from_A()，这里不会走）
                        q2_start_from_A();
                        break;

                    case Q2State::Straight_AB:
                        setBaseSpeed(0.10f);
                        driveStraightYawHold();

                        if (rising) { // 到 B
                            Prompt::once(120);
                            _pidTurn.reset();
                            q2_enter(Q2State::Arc_BC);
                        }
                        break;

                    case Q2State::Arc_BC:
                        setBaseSpeed(0.10f);
                        driveArcLineFollow(raw);

                        if (falling) { // 到 C
                            Prompt::once(120);
                            resetYawRef(); // 进入直线前重新锁航向
                            q2_enter(Q2State::Straight_CD);
                        }
                        break;

                    case Q2State::Straight_CD:
                        setBaseSpeed(0.10f);
                        driveStraightYawHold();

                        if (rising) { // 到 D
                            Prompt::once(120);
                            _pidTurn.reset();
                            q2_enter(Q2State::Arc_DA);
                        }
                        break;

                    case Q2State::Arc_DA:
                        setBaseSpeed(0.10f);
                        driveArcLineFollow(raw);

                        if (falling) { // 回到 A
                            Prompt::once(120);
                            q2_enter(Q2State::Done);
                        }
                        break;

                    case Q2State::Done:
                        setSingleMotor(_ch_L1, _ch_L2, 0.0f);
                        setSingleMotor(_ch_R1, _ch_R2, 0.0f);
                        break;
                }

                _q2_prev_hasLine = hasLine;
                break;
            }

            default:
                // 非1/2：不开车（或你想保留原case3/4逻辑可在这里继续写）
                _q1_prev_hasLine = hasLine;
                _q2_prev_hasLine = hasLine;
                break;
        }
    }
};

#ifdef __cplusplus
extern "C" {
#endif
void LineFollower_Init(void);
void LineFollower_OnTimer(void);
void LineFollower_SetSpeed(float speed);
void LineFollower_SetPID(float kp, float ki, float kd);
#ifdef __cplusplus
}
#endif

#endif