#pragma once

#include "base_control_mode.h"

/*
 * The SomeGyroPidType should specify the throttle for each motor depending on the PID.
 * The SomeDroneGyroType should abstract away the hardware of an IMU and just implement a few interface methods.
 */
template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
class TemplateDrone
{
    float _throttle = 0;
    float _yaw_desired_angle = 0;
    float _pitch_desired_angle = 0;
    float _roll_desired_angle = 0;
    BaseControlMode* _control_mode = controlModeNone();
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
        int feedback_loop_hz);

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

    void activateControlMode(BaseControlMode* control_mode);

    float getThrottle() const;

    float getDesiredYawAngle() const;

    float getDesiredPitchAngle() const;

    float getDesiredRollAngle() const;

    float getAltitude() const;

    float getLongitude() const;

    float getLatitude() const;

    float getVelocityX() const;

    float getVelocityY() const;

    float getVelocityZ() const;

    virtual void setup() = 0;

    virtual bool run() = 0;

    virtual void runMotors(float gyro_roll, float gyro_pitch, float gyro_yaw, float delta_time_seconds) = 0;

    virtual void stopMotors() = 0;

    virtual void setupMotors() = 0;

    bool updateGyro() const;

    float getYaw() const;

    float getPitch() const;

    float getRoll() const;

    float getAccelerationX() const;

    float getAccelerationY() const;

    float getAccelerationZ() const;

    void printGyro() const;

    bool hasLostConnection() const;

    void setThrottle(float value);

    void setDesiredYawAngle(float value);

    void setDesiredPitchAngle(float value);

    void setDesiredRollAngle(float value);

    virtual void enableMotors();

    virtual void disableMotors();

    bool isMotorsEnabled() const;

    BaseControlMode *getControlMode() const;

    void setControlMode(BaseControlMode *control_mode);

    unsigned long delayToKeepFeedbackLoopHz(long start_microseconds_timestamp) const;

    unsigned long timestampMicroseconds() const;

    unsigned long timestampMilliseconds() const;

    ControlMode_t getControlModeType() const;

    int getFeedbackLoopHz() const;
};

#include "template_drone.ipp"
