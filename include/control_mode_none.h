#pragma once
#include "concept_control_mode.h"

template <class U, class V, class W, class X>
class ControlModeNone
{
public:
    static ControlMode_t type() { return none; }
    static const char *name() { return "none"; }
    static bool yawCompassMode() { return false; }
    static PidConstants_t pidConstants() { return PidConstants_t{}; }
    static void activate(TemplateDroneMock<U, V, W, X> *drone) {}

    static_assert(ControlModeConcept<
        ControlModeNone,
        U, V, W, X
    >, "ControlModeNone does not satisfy ControlModeConcept");
};
