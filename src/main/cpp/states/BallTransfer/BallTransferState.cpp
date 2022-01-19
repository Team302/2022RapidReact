// C++ Includes

// FRC includes

// Team 302 includes
#include <controllers/ControlData.h>
#include <states/balltransfer/BallTransferState.h>
#include <states/Mech1MotorState.h>
#include <subsys/MechanismFactory.h>

// Third Party Includes


BallTransferState::BallTransferState
(
    ControlData*                    control,
    double                          target
) : Mech1MotorState( MechanismFactory::GetMechanismFactory()->GetBallTransfer(), control, target )
{
}