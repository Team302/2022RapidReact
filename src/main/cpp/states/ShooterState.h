#pragma once

#include <states/Mech1MotorState.h>

class ControlData;

class ShooterState : public Mech1MotorState
{
    public:

        ShooterState() = delete;
        ShooterState
        (
            ControlData*                    control,
            double                          target
        );
        ~ShooterState() = default;
};