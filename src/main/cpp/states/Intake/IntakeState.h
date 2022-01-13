#pragma once
#include <states/Mech1MotorState.h>

class ControlData;

class IntakeState : public Mech1MotorState
{
    public:

        IntakeState() = delete;
        IntakeState
        (
            ControlData* control,
            double target
        );
        ~IntakeState() = default;
};