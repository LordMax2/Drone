#pragma once

#include "pid_constants.h"
#include "concepts.h"

template <class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeProcessorType>
class TemplateDrone;

enum ControlMode_t
{
    none = 0,
    auto_level = 1,
    acro = 2
};

template <class T, class SomeGyroPidType, class SomePositionType, class SomeGyroType, class SomeHardwareProcessorType>
concept ControlModeConcept = requires(
    T mode, TemplateDrone<SomeGyroPidType, SomePositionType, SomeGyroType, SomeHardwareProcessorType>* drone)
    {
        { mode.type() } -> same_as<ControlMode_t>;

        { mode.name() } -> same_as<const char*>;

        { mode.yawCompassMode() } -> same_as<bool>;

        { mode.pidConstants() } -> same_as<PidConstants_t>;

        { mode.activate(drone) } -> same_as<void>;
    };
