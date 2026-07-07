#pragma once
#include "concept_control_mode.h"

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
class ControlModeNone
{
public:
    ControlMode_t type() { return none; }
    const char* name() { return "none"; }
    bool yawCompassMode() { return false; }
    PidConstants_t pidConstants() { return PidConstants_t{}; }

    void activate(TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>* drone)
    {
        (void)drone;
    }
};
