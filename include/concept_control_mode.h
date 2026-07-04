#pragma once

#include "pid_constants.h"
#include "concepts.h"

template <class X, class U, class V, class W>
class TemplateDroneMock;

enum ControlMode_t
{
    none = 0,
    auto_level = 1,
    acro = 2
};

template <class T, class GyroPid, class Position, class Gyro, class Hardware>
concept ControlModeConcept = requires(T mode, TemplateDroneMock<GyroPid, Position, Gyro, Hardware>* drone)
{
    { mode.type() } -> same_as<ControlMode_t>;

    { mode.name() } -> same_as<const char*>;

    { mode.yawCompassMode() } -> same_as<bool>;

    { mode.pidConstants() } -> same_as<PidConstants_t>;

    { mode.activate(drone) } -> same_as<void>;
};
