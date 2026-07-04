#pragma once

#include "pid_optimizer.h"

class Pid {
public:
    Pid(const float kp, const float ki, const float kd, const float pid_max, const int feedback_loop_hz) : _pid_max(pid_max),
                                                       _pid_optimizer(kp, ki, kd, feedback_loop_hz) {
        _kp = kp;
        _ki = ki;
        _kd = kd;
        _feedback_loop_hz = feedback_loop_hz;
    };
    virtual ~Pid() = default;

    void reset();

    void updateIntegral(float current, float desired, float delta_time_seconds = 1.0f);

    void resetIntegral();

    [[nodiscard]] float getKp() const;

    [[nodiscard]] float getKi() const;

    [[nodiscard]] float getKd() const;

    void runOptimizer(float current, float desired, long timestamp_milliseconds);

    void saveError(float current, float desired);

    float pid(float current, float desired, float delta_time_seconds = 1.0f);

    virtual float error(float current, float desired);

    float pidP(float current, float desired);
    virtual float pidD(float current, float desired, float delta_time_seconds);

    static float fconstrain(float input, float min_value, float max_value);

protected:
    [[nodiscard]] float previousError() const { return _previous_error; }

private:
    float _pid_max;
    PidOptimizer _pid_optimizer;
    float _previous_error = 0;
    float _previous_current = 0;
    float _pid_i = 0;
    float _kp;
    float _ki;
    float _kd;
    int _feedback_loop_hz;
};
