#pragma once

#include "pid.h"
#include "pid_yaw_compass.h"

class GyroPid {
public:
    GyroPid(
        const float yaw_kp, const float yaw_ki, const float yaw_kd, const bool yaw_compass_mode,
        const float pitch_kp, const float pitch_ki, const float pitch_kd,
        const float roll_kp, const float roll_ki, const float roll_kd,
        const float pid_max, const int feedback_loop_hz) : _pid_max(pid_max),
                         _feedback_loop_hz(feedback_loop_hz),
                         _pid_pitch(pitch_kp, pitch_ki, pitch_kd, pid_max, feedback_loop_hz),
                         _pid_roll(roll_kp, roll_ki, roll_kd, pid_max, feedback_loop_hz) {
        if (yaw_compass_mode) {
            _pid_yaw = new PidYawCompass(yaw_kp, yaw_ki, yaw_kd, pid_max, feedback_loop_hz);
        } else {
            _pid_yaw = new Pid(yaw_kp, yaw_ki, yaw_kd, pid_max, feedback_loop_hz);
        }
    };

    ~GyroPid() { delete _pid_yaw; }

    GyroPid(const GyroPid&) = delete;
    GyroPid& operator=(const GyroPid&) = delete;

    GyroPid(GyroPid&& other) noexcept
        : _pid_max(other._pid_max),
          _feedback_loop_hz(other._feedback_loop_hz),
          _pid_yaw(other._pid_yaw),
          _pid_pitch(other._pid_pitch),
          _pid_roll(other._pid_roll) {
        other._pid_yaw = nullptr;
    }

    GyroPid& operator=(GyroPid&& other) noexcept {
        if (this != &other) {
            delete _pid_yaw;
            _pid_max = other._pid_max;
            _feedback_loop_hz = other._feedback_loop_hz;
            _pid_yaw = other._pid_yaw;
            _pid_pitch = other._pid_pitch;
            _pid_roll = other._pid_roll;
            other._pid_yaw = nullptr;
        }
        return *this;
    }

    void reset();

    void updateIntegral(float gyro_roll, float roll_desired_angle, float gyro_pitch, float pitch_desired_angle,
                        float gyro_yaw, float yaw_desired_angle, float delta_time_seconds = 1.0f);

    void resetIntegral();

    [[nodiscard]] float getRollKp() const;

    [[nodiscard]] float getRollKi() const;

    [[nodiscard]] float getRollKd() const;

    [[nodiscard]] float getPitchKp() const;

    [[nodiscard]] float getPitchKi() const;

    [[nodiscard]] float getPitchKd() const;

    [[nodiscard]] float getYawKp() const;

    [[nodiscard]] float getYawKi() const;

    [[nodiscard]] float getYawKd() const;

    void runRollOptimizer(float gyro_roll, float roll_desired_angle, long timestamp_milliseconds);

    void runPitchOptimizer(float gyro_pitch, float pitch_desired_angle, long timestamp_milliseconds);

    void runYawOptimizer(float gyro_yaw, float yaw_desired_angle, long timestamp_milliseconds) const;

    void savePitchError(float gyro_pitch, float pitch_desired_angle);

    void saveRollError(float gyro_roll, float roll_desired_angle);

    void saveYawError(float gyro_yaw, float yaw_desired_angle) const;

    [[nodiscard]] float yawPid(float gyro_yaw, float yaw_desired_angle, float delta_time_seconds = 1.0f) const;

    float rollPid(float gyro_roll, float roll_desired_angle, float delta_time_seconds = 1.0f);

    float pitchPid(float gyro_pitch, float pitch_desired_angle, float delta_time_seconds = 1.0f);

    void setYawCompassMode(bool compass_mode) {
        const float kp = _pid_yaw->getKp();
        const float ki = _pid_yaw->getKi();
        const float kd = _pid_yaw->getKd();

        delete _pid_yaw;

        if (compass_mode) {
            _pid_yaw = new PidYawCompass(kp, ki, kd, _pid_max, _feedback_loop_hz);
        } else {
            _pid_yaw = new Pid(kp, ki, kd, _pid_max, _feedback_loop_hz);
        }
    }

private:
    float _pid_max;
    int _feedback_loop_hz;

    Pid *_pid_yaw;
    Pid _pid_pitch;
    Pid _pid_roll;
};
