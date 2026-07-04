#pragma once

#include "concepts.h"

template <class T>
concept DroneGyroConcept = requires(T gyro)
{
    { gyro.setup() } -> same_as<void>;

    { gyro.reload() } -> same_as<bool>;

    { gyro.reset() } -> same_as<void>;

    { gyro.yaw() } -> same_as<float>;

    { gyro.pitch() } -> same_as<float>;

    { gyro.roll() } -> same_as<float>;

    { gyro.accelerationX() } -> same_as<float>;

    { gyro.accelerationY() } -> same_as<float>;

    { gyro.accelerationZ() } -> same_as<float>;

    { gyro.printYawPitchRoll() } -> same_as<void>;

    { gyro.setModeAcro() } -> same_as<bool>;

    { gyro.setModeEuler() } -> same_as<bool>;

    { gyro.timestampMilliseconds() } -> same_as<unsigned long>;
};
