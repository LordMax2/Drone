#pragma once

#include "concepts.h"

template <class T>
concept HardwareProcessorConcept = requires(T processor, int milliseconds, const char *array)
{
    { processor.setup() } -> same_as<void>;

    { processor.microsecondsTimestamp() } -> same_as<unsigned long>;

    { processor.millisecondsTimestamp() } -> same_as<unsigned long>;

    { processor.sleepMilliseconds(milliseconds) } -> same_as<void>;

    { processor.print(array) } -> same_as<void>;
};
