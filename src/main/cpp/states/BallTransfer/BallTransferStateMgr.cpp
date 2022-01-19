#include <map>
#include <memory>
#include <vector>

// FRC includes
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>

// Team 302 includes
#include <states/IState.h>
#include <states/balltransfer/BallTransferStateMgr.h>
#include <xmlmechdata/StateDataDefn.h>
#include <controllers/MechanismTargetData.h>
#include <utils/Logger.h>
#include <gamepad/TeleopControl.h>
#include <states/balltransfer/BallTransferState.h>
#include <subsys/MechanismFactory.h>
#include <subsys/MechanismTypes.h>
#include <states/StateMgr.h>
#include <states/StateStruc.h>


// Third Party Includes

using namespace std;


BallTransferStateMgr* BallTransferStateMgr::m_instance = nullptr;
BallTransferStateMgr* BallTransferStateMgr::GetInstance()
{
	if ( BallTransferStateMgr::m_instance == nullptr )
	{
		BallTransferStateMgr::m_instance = new BallTransferStateMgr();
	}
	return BallTransferStateMgr::m_instance;
}


/// @brief    initialize the state manager, parse the configuration file and create the states.
BallTransferStateMgr::BallTransferStateMgr() 
{
    map<string, StateStruc> stateMap;
    stateMap["BALLTRANSFEROFF"] = m_offState;
    stateMap["BALLTRANSFERINTAKE"] = m_intakeState;
    stateMap["BALLTRANSFEREXPEL"] = m_expelState;

    Init(MechanismFactory::GetMechanismFactory()->GetBallTransfer(), stateMap);
}

/// @brief  run the current state
/// @return void
void BallTransferStateMgr::CheckForStateTransition()
{
    if ( MechanismFactory::GetMechanismFactory()->GetBallTransfer() != nullptr )
    {
        // process teleop/manual interrupts
        auto currentState = static_cast<BALL_TRANSFER_STATE>(GetCurrentState());
        
        auto controller = TeleopControl::GetInstance();
        if ( controller != nullptr )
        {
            auto intakePressed = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::INTAKE);
            auto expelPressed = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::EXPEL);
            if (intakePressed  &&  currentState != BALL_TRANSFER_STATE::INTAKE )
            {
                SetCurrentState( BALL_TRANSFER_STATE::INTAKE, false );
            }
            else if (expelPressed && currentState != BALL_TRANSFER_STATE::EXPEL )
            {
                SetCurrentState( BALL_TRANSFER_STATE::EXPEL, false );
            }           
            else if ((!intakePressed && !expelPressed) && currentState != BALL_TRANSFER_STATE::OFF )
            {
                SetCurrentState( BALL_TRANSFER_STATE::OFF, false );
            }
        }
    }
}


