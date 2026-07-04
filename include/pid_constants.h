#pragma once

struct PidConstants_t {
    PidConstants_t() : yaw_kp(0.0f), yaw_ki(0.0f), yaw_kd(0.0f),
                       pitch_kp(0.0f), pitch_ki(0.0f), pitch_kd(0.0f),
                       roll_kp(0.0f), roll_ki(0.0f), roll_kd(0.0f) {
    }

    PidConstants_t(const float yaw_kp, const float yaw_ki, const float yaw_kd,
                   const float pitch_kp, const float pitch_ki, const float pitch_kd,
                   const float roll_kp, const float roll_ki, const float roll_kd)
        : yaw_kp(yaw_kp), yaw_ki(yaw_ki), yaw_kd(yaw_kd),
          pitch_kp(pitch_kp), pitch_ki(pitch_ki), pitch_kd(pitch_kd),
          roll_kp(roll_kp), roll_ki(roll_ki), roll_kd(roll_kd) {
    }

    float yaw_kp;
    float yaw_ki;
    float yaw_kd;
    float pitch_kp;
    float pitch_ki;
    float pitch_kd;
    float roll_kp;
    float roll_ki;
    float roll_kd;

    [[nodiscard]] bool isValid() const {
        const bool everything_is_not_zero = yaw_kp != 0.0f && yaw_ki != 0.0f && yaw_kd != 0.0f &&
                                      pitch_kp != 0.0f && pitch_ki != 0.0f && pitch_kd != 0.0f &&
                                      roll_kp != 0.0f && roll_ki != 0.0f && roll_kd != 0.0f;

        return everything_is_not_zero;
    }
};
