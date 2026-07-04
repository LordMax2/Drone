#pragma once
#include "concept_control_mode.h"

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
class ControlModeNone
{
public:
    static ControlMode_t type() { return none; }
    static const char* name() { return "none"; }
    static bool yawCompassMode() { return false; }
    static PidConstants_t pidConstants() { return PidConstants_t{}; }

    static void activate(
        TemplateDroneMock<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>* drone)
    {
    }

    static_assert(ControlModeConcept<
                      ControlModeNone,
                      SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType
                  >, "ControlModeNone does not satisfy ControlModeConcept");
};
