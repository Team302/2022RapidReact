#pragma once

#include <states/Mech1MotorState.h>
#include <states/StateStruc.h>

class ControlData;

class BallTransferState : public Mech1MotorState
{
    public:

        BallTransferState() = delete;
        BallTransferState
        (
            ControlData*                    control,
            double                          target
        );
        ~BallTransferState() = default;
};