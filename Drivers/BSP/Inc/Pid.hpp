#pragma once
#include <algorithm> // 用于 std::clamp

template <typename T>
class PidController {
private:
    T _kp, _ki, _kd;
    T _min_out, _max_out;
    T _integral;
    T _last_error;
    bool _first_run;

public:
    /**
     * @brief 构造函数
     * @param kp 比例系数
     * @param ki 积分系数
     * @param kd 微分系数
     * @param min_out 输出下限
     * @param max_out 输出上限
     */
    PidController(T kp, T ki, T kd, T min_out, T max_out)
        : _kp(kp), _ki(ki), _kd(kd), 
          _min_out(min_out), _max_out(max_out), 
          _integral(0), _last_error(0), _first_run(true) {}

    /**
     * @brief 重置 PID 状态 (用于停车或重新开始时)
     */
    void reset() {
        _integral = 0;
        _last_error = 0;
        _first_run = true;
    }

    /**
     * @brief 更新 PID 参数 (用于在线调试)
     */
    void setTunings(T kp, T ki, T kd) {
        _kp = kp;
        _ki = ki;
        _kd = kd;
    }

    /**
     * @brief 计算 PID 输出
     * @param setpoint 目标值 (寻线时通常为 0)
     * @param measured 测量值 (当前的线路偏差)
     * @return T 控制量
     */
    T compute(T setpoint, T measured) {
        T error = setpoint - measured;

        // 1. 比例项
        T p_out = _kp * error;

        // 2. 积分项 (带抗饱和)
        _integral += error;
        // 简单的积分限幅，防止积分飞升
        // 更高级的做法是：如果输出饱和且误差同向，则停止积分
        T i_out = _ki * _integral;

        // 3. 微分项
        T d_out = 0;
        if (!_first_run) {
            d_out = _kd * (error - _last_error);
        } else {
            _first_run = false;
        }
        _last_error = error;

        // 4. 总输出
        T output = p_out + i_out + d_out;

        // 5. 输出限幅
        return std::clamp(output, _min_out, _max_out);
    }
};