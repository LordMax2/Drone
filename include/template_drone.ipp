#include <new>

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::TemplateDrone(
    const unsigned long transmission_timeout_definition_milliseconds,
    const int feedback_loop_hz,
    const int gyro_reset_pin) : _transmission_timeout_definition_milliseconds(
                                    transmission_timeout_definition_milliseconds),
                                _feedback_loop_hz(feedback_loop_hz),
                                pid(0, 0, 0, false, 0, 0, 0, 0, 0, 0, 0, feedback_loop_hz),
                                processor(),
                                gyro(gyro_reset_pin),
                                position(&gyro)
{
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
void TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::setPidConstants(
    float yaw_kp, float yaw_ki, float yaw_kd, bool yaw_compass_mode,
    float pitch_kp, float pitch_ki, float pitch_kd, float roll_kp,
    float roll_ki, float roll_kd)
{
    pid.~SomeGyroPidType();
    new(&pid) SomeGyroPidType(
        yaw_kp, yaw_ki, yaw_kd, yaw_compass_mode,
        pitch_kp, pitch_ki, pitch_kd,
        roll_kp, roll_ki, roll_kd,
        0, getFeedbackLoopHz());
};

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
void TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::printPid()
{
    pid.printPid(getRoll(), getDesiredRollAngle(), getPitch(), getDesiredPitchAngle(), getYaw(), getDesiredYawAngle());
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
void TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::printPidConstants()
{
    pid.printConstants();
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
void TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::resetPid()
{
    pid.reset();
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
template <typename ControlMode>
void TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::activateControlMode(
    ControlMode* control_mode)
{
    if (control_mode == nullptr)
    {
        return;
    }

    if (getControlMode() == control_mode->type())
    {
        return;
    }

    setControlMode(control_mode->type());
    control_mode->activate(this);

    const auto pid_constants = control_mode->pidConstants();

    setPidConstants(pid_constants.yaw_kp, pid_constants.yaw_ki, pid_constants.yaw_kd, control_mode->yawCompassMode(),
                    pid_constants.pitch_kp, pid_constants.pitch_ki, pid_constants.pitch_kd,
                    pid_constants.roll_kp, pid_constants.roll_ki, pid_constants.roll_kd);

    processor.print("CONTROL MODE ");
    processor.print(control_mode->name());
    processor.print("\n");
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
void TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::calculatePidIntegral(
    float gyro_roll, float gyro_pitch, float gyro_yaw,
    float delta_time_seconds)
{
    pid.updateIntegral(
        gyro_roll, getDesiredRollAngle(),
        gyro_pitch, getDesiredPitchAngle(),
        gyro_yaw, getDesiredYawAngle(),
        delta_time_seconds);
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
void TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::savePidErrors(
    float gyro_roll, float gyro_pitch, float gyro_yaw)
{
    pid.savePitchError(gyro_pitch, getDesiredPitchAngle());
    pid.saveRollError(gyro_roll, getDesiredRollAngle());
    pid.saveYawError(gyro_yaw, getDesiredYawAngle());
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
void TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::runPidOptimizer(
    long timestamp_milliseconds)
{
    pid.runRollOptimizer(getRoll(), getDesiredRollAngle(), timestamp_milliseconds);
    pid.runPitchOptimizer(getPitch(), getDesiredPitchAngle(), timestamp_milliseconds);
    pid.runYawOptimizer(getYaw(), getDesiredYawAngle(), timestamp_milliseconds);
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
void TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::runYawPidOptimizer(
    long timestamp_milliseconds)
{
    pid.runYawOptimizer(getYaw(), getDesiredYawAngle(), timestamp_milliseconds);
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
void TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::runPitchPidOptimizer(
    long timestamp_milliseconds)
{
    pid.runPitchOptimizer(getPitch(), getDesiredPitchAngle(), timestamp_milliseconds);
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
void TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::runRollPidOptimizer(
    long timestamp_milliseconds)
{
    pid.runRollOptimizer(getRoll(), getDesiredRollAngle(), timestamp_milliseconds);
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
void TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::setYawCompassMode(
    bool yaw_compass_mode)
{
    pid.setYawCompassMode(yaw_compass_mode);
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
float TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::getThrottle() const
{
    return _throttle;
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
float TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType,
                    SomeHardwareProcessorType>::getDesiredYawAngle() const
{
    return _yaw_desired_angle;
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
float TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType,
                    SomeHardwareProcessorType>::getDesiredPitchAngle() const
{
    return _pitch_desired_angle;
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
float TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType,
                    SomeHardwareProcessorType>::getDesiredRollAngle() const
{
    return _roll_desired_angle;
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
float TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::getAltitude() const
{
    return position.getAltitude();
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
float TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::getLatitude() const
{
    return position.getLatitude();
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
float TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::getLongitude() const
{
    return position.getLongitude();
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
float TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::getVelocityX() const
{
    return position.getVelocityX();
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
float TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::getVelocityY() const
{
    return position.getVelocityY();
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
float TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::getVelocityZ() const
{
    return position.getVelocityZ();
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
void TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::printGyro() const
{
    gyro.printYawPitchRoll();
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
float TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::getYaw() const
{
    return gyro.yaw();
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
float TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::getPitch() const
{
    return gyro.pitch();
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
float TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::getRoll() const
{
    return gyro.roll();
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
float TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType,
                    SomeHardwareProcessorType>::getAccelerationX() const
{
    return gyro.accelerationX();
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
float TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType,
                    SomeHardwareProcessorType>::getAccelerationY() const
{
    return gyro.accelerationY();
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
float TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType,
                    SomeHardwareProcessorType>::getAccelerationZ() const
{
    return gyro.accelerationZ();
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
bool TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType,
                   SomeHardwareProcessorType>::hasLostConnection() const
{
    const bool transmitter_lost_connection = processor.millisecondsTimestamp() - _throttle_set_timestamp >=
        _transmission_timeout_definition_milliseconds;

    return transmitter_lost_connection;
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
void TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::setThrottle(
    const float value)
{
    _throttle = value;
    _throttle_set_timestamp = processor.millisecondsTimestamp();
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
void TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::setDesiredYawAngle(
    const float value)
{
    _yaw_desired_angle = value;
    _yaw_desired_angle_set_timestamp = processor.millisecondsTimestamp();
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
void TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::setDesiredPitchAngle(
    const float value)
{
    _pitch_desired_angle = value;
    _desired_pitch_angle_set_timestamp = processor.millisecondsTimestamp();
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
void TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::setDesiredRollAngle(
    const float value)
{
    _roll_desired_angle = value;
    _desired_roll_angle_set_timestamp = processor.millisecondsTimestamp();
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
void TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::enableMotors()
{
    _is_motors_enabled = true;
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
void TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::disableMotors()
{
    _is_motors_enabled = false;
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
ControlMode_t TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType,
                            SomeHardwareProcessorType>::getControlMode() const
{
    return _current_control_mode;
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
bool TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::updateGyro() const
{
    return gyro.reload();
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
unsigned long TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType,
                            SomeHardwareProcessorType>::delayToKeepFeedbackLoopHz(
    const long start_microseconds_timestamp) const
{
    const unsigned long current_microseconds_timestamp = processor.microsecondsTimestamp();
    const long microseconds_feedback_loop_should_take = 1000000 / _feedback_loop_hz;
    const unsigned long expected_loop_duration_microseconds = static_cast<unsigned long>(
        microseconds_feedback_loop_should_take
    );
    const unsigned long elapsed_microseconds =
        current_microseconds_timestamp - static_cast<unsigned long>(start_microseconds_timestamp);

    if (elapsed_microseconds < expected_loop_duration_microseconds)
    {
        return expected_loop_duration_microseconds - elapsed_microseconds;
    }

    return 0;
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
void TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::setControlMode(
    ControlMode_t control_mode)
{
    _current_control_mode = control_mode;
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
bool TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::isMotorsEnabled() const
{
    return _is_motors_enabled;
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
unsigned long TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType,
                            SomeHardwareProcessorType>::timestampMilliseconds() const
{
    return processor.millisecondsTimestamp();
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
unsigned long TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType,
                            SomeHardwareProcessorType>::timestampMicroseconds() const
{
    return processor.microsecondsTimestamp();
}

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
int TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::getFeedbackLoopHz() const
{
    return _feedback_loop_hz;
}
