// C++ Includes

// FRC includes

// Team 302 includes
#include <controllers/ControlData.h>
#include <states/balltransfer/BallTransferState.h>
#include <states/Mech2MotorState.h>
#include <subsys/Mech2IndMotors.h>
#include <subsys/MechanismFactory.h>

// Third Party Includes


BallTransferState::BallTransferState
(
    ControlData*                    control,
    ControlData*                    control2,
    double                          primaryTarget,
    double                          secondaryTarget
) : Mech2MotorState( MechanismFactory::GetMechanismFactory()->GetBallTransfer(), control, control2, primaryTarget, secondaryTarget )
{
}