/*
*
*   This part of the drone was possbile because of my genious friend Pio Korinth. 
*   He has explained how a Black Box smiulated annealing works in a way that I could finally wrap my head around.
*
*   Thanks a lot and big credits!
*
*/

#pragma once

#include <limits.h>

#define TRIAL_DURATION_MILLISECONDS 1000

enum PidOptimizerState {
    IDLE,
    MEASURING,
    DECIDING
};

class PidOptimizer {
    float best_kp;
    float best_ki;
    float best_kd;
    long previous_score = LONG_MAX;
    int _feedback_loop_hz;

public:
    PidOptimizer(float default_kp, float default_ki, float default_kd, int feedback_loop_hz);

    virtual ~PidOptimizer() = default;

    void run(float current_error, long timestamp_milliseconds);

    float step_p = 0.001f;
    float step_i = 0.0001f;
    float step_d = 0.001f;

    float learning_rate = 0.1f;

    [[nodiscard]] float getKp() const { return current_kp; }

    [[nodiscard]] float getKi() const { return current_ki; }

    [[nodiscard]] float getKd() const { return current_kd; }

    virtual float getBestKp() { return best_kp; }

    virtual float getBestKi() { return best_ki; }

    virtual float getBestKd() { return best_kd; }

    virtual void setBestKp(float kp) { best_kp = kp; }

    virtual void setBestKi(float ki) { best_ki = ki; }

    virtual void setBestKd(float kd) { best_kd = kd; }

    float current_kp;
    float current_ki;
    float current_kd;

    virtual void setPreviousScore(long score) { previous_score = score; };

    virtual long getPreviousScore() { return previous_score; };

    PidOptimizerState state;
    unsigned long trial_start_time;
    float error_sum_squared;
    int error_measurement_count;

    void startTrial(long timestamp_milliseconds);

    [[nodiscard]] long score() const;

    virtual void evaluateTrial();

    static float fconstrain(float input, float min_value, float max_value);

    static long randomLimited(long min_value, long max_value);
};
