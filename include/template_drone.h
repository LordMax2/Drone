#pragma once

#include "concept_control_mode.h"
#include "concept_drone_position.h"
#include "concept_hardware_processor.h"
#include "concept_drone_gyro.h"

/*
 * The SomeGyroPidType should specify the throttle for each motor depending on the PID.
 * The SomeDroneGyroType should abstract away the hardware of an IMU and just implement a few interface methods.
 */
template <class SomeGyroPidType, DronePositionConcept SomePositionType, DroneGyroConcept SomeGyroType, HardwareProcessorConcept SomeHardwareProcessorType>
class TemplateDrone
{
    float _throttle = 0;
    float _yaw_desired_angle = 0;
    float _pitch_desired_angle = 0;
    float _roll_desired_angle = 0;
    ControlMode_t _current_control_mode = none;
    unsigned long _throttle_set_timestamp = 0;
    unsigned long _yaw_desired_angle_set_timestamp = 0;
    unsigned long _desired_pitch_angle_set_timestamp = 0;
    unsigned long _desired_roll_angle_set_timestamp = 0;
    bool _is_motors_enabled = false;
    unsigned long _transmission_timeout_definition_milliseconds;
    int _feedback_loop_hz;

public:
    /*
     * Create a drone
     * Default parameters that work are: 500, 200, 10000
     */
    TemplateDrone(
        unsigned long transmission_timeout_definition_milliseconds,
        int feedback_loop_hz,
        int gyro_reset_pin);

    virtual ~TemplateDrone() = default;

    SomeGyroPidType pid;
    SomeHardwareProcessorType processor;
    SomeGyroType gyro;
    SomePositionType position;

    void printPid();

    void printPidConstants();

    void resetPid();

    void calculatePidIntegral(float gyro_roll, float gyro_pitch, float gyro_yaw, float delta_time_seconds = 1.0f);

    void savePidErrors(float gyro_roll, float gyro_pitch, float gyro_yaw);

    void runPidOptimizer(long timestamp_milliseconds);

    void runYawPidOptimizer(long timestamp_milliseconds);

    void runPitchPidOptimizer(long timestamp_milliseconds);

    void runRollPidOptimizer(long timestamp_milliseconds);

    /*
     * Yaw compass mode means that the yaw PID will be able to translate the gyroscope yaw angle to a compass angle, because they differ a bit, IMU output is usually -180 to +180, which makes things a bit math:ey.
     * Use this only when you use something like auto-level mode, and not in racing angle-rate mote.
     */
    void setYawCompassMode(bool yaw_compass_mode);

    void setPidConstants(float yaw_kp, float yaw_ki, float yaw_kd, bool yaw_compass_mode, float pitch_kp,
                         float pitch_ki, float pitch_kd, float roll_kp, float roll_ki, float roll_kd);

    template <typename ControlMode>
    void activateControlMode(ControlMode* control_mode);

    [[nodiscard]] float getThrottle() const;

    [[nodiscard]] float getDesiredYawAngle() const;

    [[nodiscard]] float getDesiredPitchAngle() const;

    [[nodiscard]] float getDesiredRollAngle() const;

    [[nodiscard]] float getAltitude();

    [[nodiscard]] float getLongitude();

    [[nodiscard]] float getLatitude();

    [[nodiscard]] float getVelocityX();

    [[nodiscard]] float getVelocityY();

    [[nodiscard]] float getVelocityZ();

    [[nodiscard]] bool updateGyro();

    [[nodiscard]] float getYaw();

    [[nodiscard]] float getPitch();

    [[nodiscard]] float getRoll();

    [[nodiscard]] float getAccelerationX();

    [[nodiscard]] float getAccelerationY();

    [[nodiscard]] float getAccelerationZ();

    void printGyro();

    [[nodiscard]] bool hasLostConnection() const;

    void setThrottle(float value);

    void setDesiredYawAngle(float value);

    void setDesiredPitchAngle(float value);

    void setDesiredRollAngle(float value);

    virtual void enableMotors();

    virtual void disableMotors();

    [[nodiscard]] bool isMotorsEnabled() const;

    [[nodiscard]] ControlMode_t getControlMode() const;

    void setControlMode(ControlMode_t control_mode);

    [[nodiscard]] unsigned long delayToKeepFeedbackLoopHz(long start_microseconds_timestamp);

    [[nodiscard]] unsigned long timestampMicroseconds();

    [[nodiscard]] unsigned long timestampMilliseconds();

    [[nodiscard]] int getFeedbackLoopHz() const;
};

#include "template_drone.ipp"
