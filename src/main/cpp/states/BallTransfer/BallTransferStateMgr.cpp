//====================================================================================================================================================
// Copyright 2022 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

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
    stateMap["BALLTRANSFERLIFT"] = m_liftState;
    stateMap["BALLTRANSFERSPIN"] = m_liftState;

    Init(MechanismFactory::GetMechanismFactory()->GetBallTransfer(), stateMap);
}

/// @brief  run the current state
/// @return void
void BallTransferStateMgr::CheckForStateTransition()
{
    /**
    if ( MechanismFactory::GetMechanismFactory()->GetBallTransfer() != nullptr )
    {
        // process teleop/manual interrupts
        auto currentState = static_cast<BALL_TRANSFER_STATE>(GetCurrentState());
        
        auto controller = TeleopControl::GetInstance();
        if ( controller != nullptr )
        {
            auto spinPressed = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::BALL_TRANSFER_SPIN);
            auto liftPressed = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::BALL_TRANSFER_LIFT);
            if (spinPressed  &&  currentState != BALL_TRANSFER_STATE::SPIN )
            {
                SetCurrentState( BALL_TRANSFER_STATE::SPIN, false );
            }
            else if (liftPressed && currentState != BALL_TRANSFER_STATE::LIFT )
            {
                SetCurrentState( BALL_TRANSFER_STATE::LIFT, false );
            }           
            else if ((!spinPressed && !liftPressed) && currentState != BALL_TRANSFER_STATE::OFF )
            {
                SetCurrentState( BALL_TRANSFER_STATE::OFF, false );
            }
        }
    }
    **/
}


