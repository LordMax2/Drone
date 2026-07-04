#pragma once

#include "concepts.h"

template <class T>
concept DronePositionConcept = requires(T position)
{
    { position.getAltitude() } -> same_as<float>;

    { position.getLongitude() } -> same_as<float>;

    { position.getLatitude() } -> same_as<float>;

    { position.getVelocityX() } -> same_as<float>;

    { position.getVelocityY() } -> same_as<float>;

    { position.getVelocityZ() } -> same_as<float>;
};
