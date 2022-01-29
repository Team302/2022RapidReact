#pragma once

#include <states/Mech2MotorState.h>
#include <states/StateStruc.h>

class ControlData;

class BallTransferState : public Mech2MotorState
{
    public:

        BallTransferState() = delete;
        BallTransferState
        (
            ControlData*                    control,
            ControlData*                    control2,
            double                          primaryTarget,
            double                          secondaryTarget
        );
        ~BallTransferState() = default;
};