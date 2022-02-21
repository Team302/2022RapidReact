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
#include <subsys/MechanismFactory.h>
#include <subsys/MechanismTypes.h>
#include <xmlmechdata/StateDataDefn.h>


// Third Party Includes

using namespace std;

ClimberStateMgr* ClimberStateMgr::m_instance = nullptr;
ClimberStateMgr* ClimberStateMgr::GetInstance()
{
	if ( ClimberStateMgr::m_instance == nullptr )
	{
        // Create the shooter state manager, ball transfer state manager, turret state manager and hopper state manager
		ClimberStateMgr::m_instance = new ClimberStateMgr();
    }
	return ClimberStateMgr::m_instance;
}

/// @brief    initialize the state manager, parse the configuration file and create the states.
ClimberStateMgr::ClimberStateMgr()    
{
    // initialize the xml string to state map
    map<string, StateStruc> stateMap;
    stateMap["CLIMBEROFF"] = m_offState;
    stateMap["CLIMBERINITIALREACH"] = m_initialReachState;
    stateMap["CLIMBERRETRACT"] = m_retractState;
    stateMap["CLIMBERRELEASE"] = m_releaseState;
    stateMap["CLIMBERREACHTOBAR"] = m_reachToBarState;
    stateMap["CLIMBERROTATEOUT"] = m_rotateOutState;
    stateMap["CLIMBERROTATEIN"] = m_rotateInState;
    stateMap["CLIMBERHOLD"] = m_holdState;

    Init(MechanismFactory::GetMechanismFactory()->GetClimber(), stateMap);
}

/// @brief run the current state
/// @return void
void ClimberStateMgr::CheckForStateTransition()
{
    //Check for button input
}