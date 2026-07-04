#pragma once

#include "concepts.h"

template <class T>
concept DroneMotorConcept = requires(T motor, float percentage)
{
    { motor.setSpeed(percentage) } -> same_as<void>;
};
