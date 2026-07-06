#include "Arduino.h"
#include "concept_drone_gyro.h"
#include "concept_drone_motor.h"
#include "concept_drone_position.h"
#include "concept_hardware_processor.h"
#include "gyro_pid.h"
#include "template_drone.h"
#include "pid_constants.h"

class MockHardwareProcessor
{
public:
    void setup() {}

    static unsigned long microsecondsTimestamp() { return micros(); }
    static unsigned long millisecondsTimestamp() { return millis(); }
    static void sleepMilliseconds(const int) {}
    static void print(const char*) {}
};

static_assert(HardwareProcessorConcept<MockHardwareProcessor>);

class MockDroneGyro
{
public:
    explicit MockDroneGyro(const int) {}

    void setup() {}
    bool reload() const { return true; }
    void reset() {}
    float yaw() const { return 0.0f; }
    float pitch() const { return 0.0f; }
    float roll() const { return 0.0f; }
    float accelerationX() const { return 0.0f; }
    float accelerationY() const { return 0.0f; }
    float accelerationZ() const { return 0.0f; }
    void printYawPitchRoll() const {}
    bool setModeAcro() { return true; }
    bool setModeEuler() { return true; }
    unsigned long timestampMilliseconds() const { return millis(); }
};

static_assert(DroneGyroConcept<MockDroneGyro>);

class MockDronePosition
{
public:
    explicit MockDronePosition(MockDroneGyro*) {}

    float getAltitude() const { return 0.0f; }
    float getLongitude() const { return 0.0f; }
    float getLatitude() const { return 0.0f; }
    float getVelocityX() const { return 0.0f; }
    float getVelocityY() const { return 0.0f; }
    float getVelocityZ() const { return 0.0f; }
};

static_assert(DronePositionConcept<MockDronePosition>);

class MockDroneMotor
{
public:
    void setSpeed(const float) {}
};

static_assert(DroneMotorConcept<MockDroneMotor>);

class MockGyroPid : public GyroPid
{
public:
    using GyroPid::GyroPid;

    void printPid(
        const float, const float, const float, const float, const float, const float) const
    {
    }

    void printConstants() const {}
};

template <
    class SomeGyroPidType, class SomePositionType,
    class SomeGyroType, class SomeHardwareProcessorType>
class TemplateDroneMock
    : public TemplateDrone<
          SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>
{
    using Base = TemplateDrone<
        SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>;

public:
    TemplateDroneMock(
        const unsigned long transmission_timeout_ms,
        const int feedback_loop_hz,
        const int gyro_reset_pin)
        : Base(transmission_timeout_ms, feedback_loop_hz, gyro_reset_pin)
    {
    }

    void setup() override;

    bool run() override { return false; }

    void runMotors(const float, const float, const float, const float) override {}

    void stopMotors() override {}

    void setupMotors() override {}
};

#include "control_mode_none.h"

template <class SomeGyroPidType, class SomePositionType,    class SomeGyroType, class SomeHardwareProcessorType>
void TemplateDroneMock<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>::setup()
{
    using NoneMode = ControlModeNone<
        SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>;
    static NoneMode none_mode;

    Base::setControlMode(none_mode.type());
    none_mode.activate(this);

    const auto pid_constants = none_mode.pidConstants();
    Base::setPidConstants(
        pid_constants.yaw_kp, pid_constants.yaw_ki, pid_constants.yaw_kd,
        none_mode.yawCompassMode(),
        pid_constants.pitch_kp, pid_constants.pitch_ki, pid_constants.pitch_kd,
        pid_constants.roll_kp, pid_constants.roll_ki, pid_constants.roll_kd);
}

using MockDrone = TemplateDroneMock<MockGyroPid, MockDronePosition, MockDroneGyro, MockHardwareProcessor>;

static MockDrone drone(500, 200, 10);

static void validateTemplateDroneApi(MockDrone& d)
{
    d.setup();

    d.setThrottle(50.0f);
    d.setDesiredYawAngle(1.0f);
    d.setDesiredPitchAngle(2.0f);
    d.setDesiredRollAngle(3.0f);
    d.setControlMode(acro);

    d.printPid();
    d.printPidConstants();
    d.resetPid();
    d.calculatePidIntegral(0.0f, 0.0f, 0.0f, 0.01f);
    d.savePidErrors(0.0f, 0.0f, 0.0f);

    const long now = static_cast<long>(d.timestampMilliseconds());
    d.runPidOptimizer(now);
    d.runYawPidOptimizer(now);
    d.runPitchPidOptimizer(now);
    d.runRollPidOptimizer(now);

    d.setYawCompassMode(true);
    d.setPidConstants(1.0f, 0.0f, 0.0f, true, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f);

    (void)d.getThrottle();
    (void)d.getDesiredYawAngle();
    (void)d.getDesiredPitchAngle();
    (void)d.getDesiredRollAngle();
    (void)d.getAltitude();
    (void)d.getLongitude();
    (void)d.getLatitude();
    (void)d.getVelocityX();
    (void)d.getVelocityY();
    (void)d.getVelocityZ();
    (void)d.getYaw();
    (void)d.getPitch();
    (void)d.getRoll();
    (void)d.getAccelerationX();
    (void)d.getAccelerationY();
    (void)d.getAccelerationZ();
    (void)d.getControlMode();
    (void)d.getFeedbackLoopHz();

    (void)d.updateGyro();
    d.printGyro();
    (void)d.hasLostConnection();
    d.enableMotors();
    d.disableMotors();
    (void)d.isMotorsEnabled();
    (void)d.timestampMicroseconds();
    (void)d.timestampMilliseconds();
    (void)d.delayToKeepFeedbackLoopHz(0);

    d.runMotors(0.0f, 0.0f, 0.0f, 0.01f);
    d.stopMotors();
    d.setupMotors();
    (void)d.run();
}

void setup()
{
    validateTemplateDroneApi(drone);
}

void loop()
{
}
