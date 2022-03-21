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

// C++ Includes
#include <map>
#include <memory>
#include <vector>

// FRC includes
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>

// Team 302 includes
#include <controllers/MechanismTargetData.h>
#include <gamepad/TeleopControl.h>
#include <states/climber/ClimberState.h>
#include <states/climber/ClimberStateMgr.h>
#include <states/IState.h>
#include <subsys/Climber.h>
#include <subsys/MechanismFactory.h>
#include <subsys/MechanismTypes.h>
#include <utils/Logger.h>
#include <xmlmechdata/StateDataDefn.h>


// Third Party Includes

using namespace std;

ClimberStateMgr* ClimberStateMgr::m_instance = nullptr;
ClimberStateMgr* ClimberStateMgr::GetInstance()
{
	if ( ClimberStateMgr::m_instance == nullptr )
	{
	    auto mechFactory = MechanismFactory::GetMechanismFactory();
	    auto climber = mechFactory->GetClimber();
	    if (climber != nullptr)
        {
		    ClimberStateMgr::m_instance = new ClimberStateMgr();
        }
    }
	return ClimberStateMgr::m_instance;
}

/// @brief    initialize the state manager, parse the configuration file and create the states.
ClimberStateMgr::ClimberStateMgr() : m_climber(MechanismFactory::GetMechanismFactory()->GetClimber()),
                                     m_nt()
{
    if (m_climber != nullptr)
    {
        auto ntName = m_climber->GetNetworkTableName();
        m_nt = nt::NetworkTableInstance::GetDefault().GetTable(ntName);
    }
    else
    {
        m_nt = nt::NetworkTableInstance::GetDefault().GetTable("climber");
    }    
    
    // initialize the xml string to state map
    map<string, StateStruc> stateMap;
    stateMap["CLIMBER_OFF"] = m_offState;
    stateMap["CLIMBER_MANUAL"] = m_manualState;
    stateMap["CLIMBER_INITIALREACH"] = m_initialReachState;
    stateMap["CLIMBER_RETRACT"] = m_retractState;
    stateMap["CLIMBER_RELEASE"] = m_releaseState;
    stateMap["CLIMBER_REACHTOBAR"] = m_reachToBarState;
    stateMap["CLIMBER_ROTATEOUT"] = m_rotateOutState;
    stateMap["CLIMBER_ROTATEIN"] = m_rotateInState;
    stateMap["CLIMBER_HOLD"] = m_holdState;

    Init(m_climber, stateMap);
}

/// @brief run the current state
/// @return void
void ClimberStateMgr::CheckForStateTransition()
{
    if (m_climber != nullptr )
    {
        // process teleop/manual interrupts
        auto currentState = static_cast<CLIMBER_STATE>(GetCurrentState());
        auto targetState = currentState;
        Logger::GetLogger()->ToNtTable(m_nt, string("Current climber State"), currentState);
        auto controller = TeleopControl::GetInstance();
        auto isClimbMode  = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::SELECT_CLIMBER_ARM) : false;
        if (isClimbMode)
        {
            //auto armDownSpeed = controller != nullptr ? controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::CLIMBER_MAN_DOWN) : 0.0;
            //auto armUpSpeed   = controller != nullptr ? controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::CLIMBER_MAN_UP) : 0.0;
            //auto armRotateSpeed = controller != nullptr ? controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::CLIMBER_MAN_ROTATE) : 0.0;

            //auto isManual = abs(armDownSpeed) > 0.1 || abs(armUpSpeed) > 0.1 || abs(armRotateSpeed) > 0.1;
            //targetState = isManual ? CLIMBER_STATE::MANUAL : CLIMBER_STATE::OFF;
            targetState = CLIMBER_STATE::MANUAL;
        }
        else
        {
            targetState = CLIMBER_STATE::OFF;
        }

        if (targetState != currentState)
        {
            Logger::GetLogger()->ToNtTable(m_nt, string("Changing climber State"), targetState);
            SetCurrentState(targetState, true);
        }
    }
}