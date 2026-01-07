#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include "main.h"
#include "Pid.hpp"
#include <cmath>

#include "PidStorage.hpp"
#include "Prompt.hpp"
#include "DbgSerial.hpp"

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

    float _q2_yaw_A_deg = 0.0f;

    // ====== Arc 段丢线保持方向 ======
    float _arc_last_turn = 0.0f;   // 上一次有效循线的 turn_adjust（决定方向）
    uint8_t _arc_lost_cnt = 0;     // 连续丢线次数（20ms一次）

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

        if (raw & GPIO_PIN_15) { sum += -4.0f; count++; }
        if (raw & GPIO_PIN_12) { sum += -2.0f; count++; }
        if (raw & GPIO_PIN_11) { sum +=  0.0f; count++; }
        if (raw & GPIO_PIN_10) { sum +=  2.0f; count++; }
        if (raw & GPIO_PIN_8)  { sum +=  4.0f; count++; }

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

    // ====== 半圆段：循线（Arc段）======
    // 优化：短暂丢线不停车，沿用上一次转向方向保持行进（baseSpeed=0.2适配）
    void driveArcLineFollow(uint16_t raw) {
        float position_error = 0.0f;

        if (calcPositionError(raw, position_error)) {
            // 有线：正常循线
            _arc_lost_cnt = 0;

            float turn_adjust = _pidTurn.compute(0.0f, position_error);

            // 限幅：base=0.2时建议不要超过0.35，避免过激
            if (turn_adjust > 0.35f) turn_adjust = 0.35f;
            if (turn_adjust < -0.35f) turn_adjust = -0.35f;

            _arc_last_turn = turn_adjust;
            setEndSpeed(turn_adjust, 0.0f);
            return;
        }

        // 丢线：不停车，保持上次方向继续走
        if (_arc_lost_cnt < 250) _arc_lost_cnt++;

        // 固定保持幅度（不会让轮子反转）
        float keep = 0.0f;
        if (_arc_last_turn > 0.02f) keep = 0.12f;
        else if (_arc_last_turn < -0.02f) keep = -0.12f;
        else keep = 0.0f;

        // 兜底：丢线太久（比如>1.0s）认为跑飞，停下更安全
        // 20ms周期下 50次≈1.0s
        if (_arc_lost_cnt > 50) {
            setSingleMotor(_ch_L1, _ch_L2, 0.0f);
            setSingleMotor(_ch_R1, _ch_R2, 0.0f);
            return;
        }

        setEndSpeed(keep, 0.0f);
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

    // Q2 边沿去抖时间戳（ms）
    uint32_t _q2_last_edge_ms = 0;
    // 新增：Arc 段丢线稳定计数（防抖）
    uint8_t _q2_noLine_cnt = 0;
    uint8_t _q2_hasLine_cnt = 0; // 可选：如果你后面想做“有线稳定N次”

    void q2_enter(Q2State s) {
        _q2_state = s;
        _q2_prompted = false;

        _q2_noLine_cnt = 0;
        _q2_hasLine_cnt = 0;

        _arc_lost_cnt = 0;
        // _arc_last_turn 可选择保留或清零：清零更安全（默认向前走）
        _arc_last_turn = 0.0f;

        const char* name = "?";
        switch (s) {
            case Q2State::Idle:        name = "Idle"; break;
            case Q2State::Straight_AB: name = "Straight_AB"; break;
            case Q2State::Arc_BC:      name = "Arc_BC"; break;
            case Q2State::Straight_CD: name = "Straight_CD"; break;
            case Q2State::Arc_DA:      name = "Arc_DA"; break;
            case Q2State::Done:        name = "Done"; break;
            default: break;
        }

        uint16_t raw = (GPIOA->IDR) & LF_SENSOR_MASK;
        g_dbgTx3.printf("[Q2] -> %s raw=0x%04X yaw=%.2f\r\n", name, raw, User_YPR[0]);
    }

    // ================== Q3 状态机：A->C->B->D->A ==================
    // 说明：你的地图为矩形 A,B,C,D 顺时针；B->C 与 D->A 是半圆段（循线）
    // 本题路径拆解：
    // A->C：直线（锁航向）
    // C->B：半圆（循线）
    // B->D：直线（锁航向）
    // D->A：半圆（循线）

    enum class Q3State : uint8_t { Idle=0, Straight_AC, Arc_CB, Straight_BD, Arc_DA, Done };
    Q3State _q3_state = Q3State::Idle;
    bool _q3_prev_hasLine = false;
    bool _q3_prompted = false;

    uint32_t _q3_last_edge_ms = 0;
    uint8_t _q3_noLine_cnt = 0;

    void q3_enter(Q3State s) {
        _q3_state = s;
        _q3_prompted = false;
        _q3_noLine_cnt = 0;

        _arc_lost_cnt = 0;
        _arc_last_turn = 0.0f;

        const char* name = "?";
        switch (s) {
            case Q3State::Idle:        name = "Idle"; break;
            case Q3State::Straight_AC: name = "Straight_AC"; break;
            case Q3State::Arc_CB:      name = "Arc_CB"; break;
            case Q3State::Straight_BD: name = "Straight_BD"; break;
            case Q3State::Arc_DA:      name = "Arc_DA"; break;
            case Q3State::Done:        name = "Done"; break;
            default: break;
        }

        uint16_t raw = (GPIOA->IDR) & LF_SENSOR_MASK;
        g_dbgTx3.printf("[Q3] -> %s raw=0x%04X yaw=%.2f\r\n", name, raw, User_YPR[0]);
    }

    // ================== Q4 状态机：按 Q3 路径跑 4 圈后停车 ==================
    enum class Q4State : uint8_t { Idle=0, Straight_AC, Arc_CB, Straight_BD, Arc_DA, Done };
    Q4State _q4_state = Q4State::Idle;

    bool _q4_prev_hasLine = false;
    uint32_t _q4_last_edge_ms = 0;
    uint8_t _q4_noLine_cnt = 0;

    uint8_t _q4_lap = 0;                 // 已完成圈数
    static constexpr uint8_t Q4_LAPS = 4;

    void q4_enter(Q4State s) {
        _q4_state = s;
        _q4_noLine_cnt = 0;

        _arc_lost_cnt = 0;
        _arc_last_turn = 0.0f;

        const char* name = "?";
        switch (s) {
            case Q4State::Idle:        name = "Idle"; break;
            case Q4State::Straight_AC: name = "Straight_AC"; break;
            case Q4State::Arc_CB:      name = "Arc_CB"; break;
            case Q4State::Straight_BD: name = "Straight_BD"; break;
            case Q4State::Arc_DA:      name = "Arc_DA"; break;
            case Q4State::Done:        name = "Done"; break;
            default: break;
        }

        uint16_t raw = (GPIOA->IDR) & LF_SENSOR_MASK;
        g_dbgTx3.printf("[Q4] -> %s lap=%u raw=0x%04X yaw=%.2f\r\n", name, (unsigned)_q4_lap, raw, User_YPR[0]);
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
        if (q != 3) { _q3_state = Q3State::Idle; _q3_prompted = false; }
        if (q != 4) { _q4_state = Q4State::Idle; _q4_lap = 0; }
    }

    // Q2 显式启动：进第二题时在 A 点提示一次
    void q2_start_from_A() {
        // 你保证起跑 raw==0，所以 prev_hasLine=false 合理
        _q2_prev_hasLine = false;
        _q2_last_edge_ms = HAL_GetTick(); // 新增：启动时记一下，避免刚起步就误触发
        _q2_yaw_A_deg = User_YPR[0];
        resetYawRef();
        q2_enter(Q2State::Straight_AB);
        // A 点提示一次
        Prompt::once(120);
        _q2_prompted = true;
    }

    // Q3 显式启动：进第三题时在 A 点提示一次
    void q3_start_from_A() {
        _q3_prev_hasLine = false;
        _q3_last_edge_ms = HAL_GetTick(); // 避免刚切题误触发
        resetYawRef();
        q3_enter(Q3State::Straight_AC);

        Prompt::once(120);   // A 点提示一次
        _q3_prompted = true;
    }

    // Q4 显式启动：从 A 点开始，提示一次，然后开始第 1 圈
    void q4_start_from_A() {
        _q4_prev_hasLine = false;
        _q4_last_edge_ms = HAL_GetTick();
        _q4_noLine_cnt = 0;

        _q4_lap = 0;
        resetYawRef();
        q4_enter(Q4State::Straight_AC);

        Prompt::once(120); // A 点提示一次（起步）
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
                        setBaseSpeed(0.30f);
                        resetYawRef();
                        q1_enter(Q1State::GoStraight_AB);
                        break;

                    case Q1State::GoStraight_AB:
                        setBaseSpeed(0.30f);
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
                // ===== Q2: 边沿去抖 + Arc段 falling 稳定判定 =====
                const uint32_t DEBOUNCE_MS = 200;
                uint32_t now = HAL_GetTick();
                bool edge_ok = (now - _q2_last_edge_ms) >= DEBOUNCE_MS;

                // 原始边沿（不稳定）
                bool rising_raw  = (!_q2_prev_hasLine) && hasLine;
                bool falling_raw = (_q2_prev_hasLine) && (!hasLine);

                // 1) rising：仍然用全局去抖（避免B点连触发）
                bool rising = edge_ok && rising_raw;

                // 2) falling：在 Arc 段要求“连续丢线 N 次”才算（防止刚进B->C就短暂丢线）
                bool falling = false;
                constexpr uint8_t NO_LINE_N = 20; // 20ms周期下：约140ms丢线才算真正到点

                if (_q2_state == Q2State::Arc_BC || _q2_state == Q2State::Arc_DA) {
                    if (!hasLine) {
                        if (_q2_noLine_cnt < 255) _q2_noLine_cnt++;
                    } else {
                        _q2_noLine_cnt = 0;
                    }
                    falling = edge_ok && (_q2_noLine_cnt >= NO_LINE_N);
                } else {
                    // 直线段 falling 不用（直线段只看 rising 到B/D）
                    falling = edge_ok && falling_raw;
                }

                // 如果本次确认了有效边沿，更新时间戳，并清一下计数器（避免下一拍继续触发）
                if (rising || falling) {
                    _q2_last_edge_ms = now;
                    _q2_noLine_cnt = 0;
                    _q2_hasLine_cnt = 0;
                }

                switch (_q2_state) {
                    case Q2State::Idle:
                        // 兜底启动（如果你在外层已经调用 q2_start_from_A()，这里不会走）
                        q2_start_from_A();
                        break;

                    case Q2State::Straight_AB:
                        setBaseSpeed(0.30f);
                        driveStraightYawHold();

                        if (rising) { // 到 B
                            Prompt::once(120);
                            _pidTurn.reset();
                            q2_enter(Q2State::Arc_BC);
                        }
                        break;

                    case Q2State::Arc_BC:
                        setBaseSpeed(0.20f);
                        driveArcLineFollow(raw);

                        if (falling) { // 到 C
                            Prompt::once(120);
                            //resetYawRef(); // 进入直线前重新锁航向
                            // C 点出来：直走目标角 = A 点出发角 + 178°，并归一化到 [-180, 180]
                            float target = _q2_yaw_A_deg + 178.0f;
                            while (target > 180.0f) target -= 360.0f;
                            while (target < -180.0f) target += 360.0f;
                            setYawRefDeg(target);
                            q2_enter(Q2State::Straight_CD);
                        }
                        break;

                    case Q2State::Straight_CD:
                        setBaseSpeed(0.30f);
                        driveStraightYawHold();

                        if (rising) { // 到 D
                            Prompt::once(120);
                            _pidTurn.reset();
                            q2_enter(Q2State::Arc_DA);
                        }
                        break;

                    case Q2State::Arc_DA:
                        setBaseSpeed(0.20f);
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

            case 3: {
                // Q3: A->C->B->D->A，每过点提示一次（状态机）
                const uint32_t DEBOUNCE_MS = 200;
                uint32_t now = HAL_GetTick();
                bool edge_ok = (now - _q3_last_edge_ms) >= DEBOUNCE_MS;

                bool rising_raw = (!_q3_prev_hasLine) && hasLine; // 无线->有线（到点：B 或 D）
                bool falling_raw = (_q3_prev_hasLine) && (!hasLine); // 有线->无线（到点：C 或 A）

                bool rising = edge_ok && rising_raw;

                bool falling = false;
                constexpr uint8_t NO_LINE_N = 20; // 20ms周期下约140ms丢线才认为真正到点

                // 在半圆段用“连续丢线N次”判定 falling，更稳
                if (_q3_state == Q3State::Arc_CB || _q3_state == Q3State::Arc_DA) {
                    if (!hasLine) {
                        if (_q3_noLine_cnt < 255) _q3_noLine_cnt++;
                    } else {
                        _q3_noLine_cnt = 0;
                    }
                    falling = edge_ok && (_q3_noLine_cnt >= NO_LINE_N);
                } else {
                    falling = edge_ok && falling_raw;
                }

                if (rising || falling) {
                    _q3_last_edge_ms = now;
                    _q3_noLine_cnt = 0;
                }

                switch (_q3_state) {
                    case Q3State::Idle:
                        // 兜底启动（正常情况下在切到Q3时外层会调用 q3_start_from_A）
                        q3_start_from_A();
                        break;

                    case Q3State::Straight_AC:
                        setBaseSpeed(0.30f);
                        driveStraightYawHold();
                        if (falling) {
                            // 到 C（有线->无线）
                            Prompt::once(120);
                            _pidTurn.reset();
                            q3_enter(Q3State::Arc_CB);
                        }
                        break;

                    case Q3State::Arc_CB:
                        setBaseSpeed(0.20f);
                        driveArcLineFollow(raw);
                        if (rising) {
                            // 到 B（无线->有线）
                            Prompt::once(120);
                            resetYawRef();
                            q3_enter(Q3State::Straight_BD);
                        }
                        break;

                    case Q3State::Straight_BD:
                        setBaseSpeed(0.30f);
                        driveStraightYawHold();
                        if (rising) {
                            // 到 D（无线->有线）
                            Prompt::once(120);
                            _pidTurn.reset();
                            q3_enter(Q3State::Arc_DA);
                        }
                        break;

                    case Q3State::Arc_DA:
                        setBaseSpeed(0.20f);
                        driveArcLineFollow(raw);
                        if (falling) {
                            // 回到 A（有线->无线）
                            Prompt::once(120);
                            q3_enter(Q3State::Done);
                        }
                        break;

                    case Q3State::Done:
                        setSingleMotor(_ch_L1, _ch_L2, 0.0f);
                        setSingleMotor(_ch_R1, _ch_R2, 0.0f);
                        break;
                }

                _q3_prev_hasLine = hasLine;
                break;
            }

            case 4: {
                // Q4: 按 Q3 路径跑 4 圈后停车
                const uint32_t DEBOUNCE_MS = 200;
                uint32_t now = HAL_GetTick();
                bool edge_ok = (now - _q4_last_edge_ms) >= DEBOUNCE_MS;

                bool rising_raw = (!_q4_prev_hasLine) && hasLine; // 无线->有线：B 或 D
                bool falling_raw = (_q4_prev_hasLine) && (!hasLine); // 有线->无线：C 或 A

                bool rising = edge_ok && rising_raw;

                bool falling = false;
                constexpr uint8_t NO_LINE_N = 20;
                if (_q4_state == Q4State::Arc_CB || _q4_state == Q4State::Arc_DA) {
                    if (!hasLine) {
                        if (_q4_noLine_cnt < 255) _q4_noLine_cnt++;
                    } else {
                        _q4_noLine_cnt = 0;
                    }
                    falling = edge_ok && (_q4_noLine_cnt >= NO_LINE_N);
                } else {
                    falling = edge_ok && falling_raw;
                }

                if (rising || falling) {
                    _q4_last_edge_ms = now;
                    _q4_noLine_cnt = 0;
                }

                switch (_q4_state) {
                    case Q4State::Idle:
                        q4_start_from_A();
                        break;

                    case Q4State::Straight_AC:
                        setBaseSpeed(0.30f);
                        driveStraightYawHold();
                        if (falling) {
                            // 到 C
                            Prompt::once(120);
                            _pidTurn.reset();
                            q4_enter(Q4State::Arc_CB);
                        }
                        break;

                    case Q4State::Arc_CB:
                        setBaseSpeed(0.20f);
                        driveArcLineFollow(raw);
                        if (rising) {
                            // 到 B
                            Prompt::once(120);
                            resetYawRef();
                            q4_enter(Q4State::Straight_BD);
                        }
                        break;

                    case Q4State::Straight_BD:
                        setBaseSpeed(0.30f);
                        driveStraightYawHold();
                        if (rising) {
                            // 到 D
                            Prompt::once(120);
                            _pidTurn.reset();
                            q4_enter(Q4State::Arc_DA);
                        }
                        break;

                    case Q4State::Arc_DA:
                        setBaseSpeed(0.20f);
                        driveArcLineFollow(raw);
                        if (falling) {
                            // 回到 A：计圈
                            Prompt::once(120);

                            _q4_lap++;
                            if (_q4_lap >= Q4_LAPS) {
                                q4_enter(Q4State::Done);
                            } else {
                                // 下一圈从 A->C 继续
                                resetYawRef();
                                q4_enter(Q4State::Straight_AC);
                            }
                        }
                        break;

                    case Q4State::Done:
                        setSingleMotor(_ch_L1, _ch_L2, 0.0f);
                        setSingleMotor(_ch_R1, _ch_R2, 0.0f);
                        break;
                }

                _q4_prev_hasLine = hasLine;
                break;
            }

            default:
                // 非1/2/3：不开车（或你想保留原case3/4逻辑可在这里继续写）
                _q1_prev_hasLine = hasLine;
                _q2_prev_hasLine = hasLine;
                _q3_prev_hasLine = hasLine;
                _q4_prev_hasLine = hasLine;
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