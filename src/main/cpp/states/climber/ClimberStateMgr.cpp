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
                                     m_nt(),
                                     m_wasAutoClimb(false),
                                     m_prevState(CLIMBER_STATE::OFF)
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
    stateMap[m_climberOffXmlString] = m_offState;
    stateMap[m_climberManualXmlString] = m_manualState;
    stateMap[m_climberInitialReachXmlString] = m_initialReachState;
    stateMap[m_climberClimbMidXmlString] = m_climbMidState;
    stateMap[m_climberExtendMidXmlString] = m_extendMidState;
    stateMap[m_climberRotateMidXmlString] = m_rotateMidState;
    stateMap[m_climberReachHighXmlString] = m_reachHighState;
    stateMap[m_climberClimbHighXmlString] = m_climbHighState;
    stateMap[m_climberExtendHighXmlString] = m_extendHighState;
    stateMap[m_climberClimbTraversalXmlString] = m_climbTraversalState;

    Init(m_climber, stateMap);
}

/// @brief run the current state
/// @return void
void ClimberStateMgr::CheckForStateTransition()
{
    auto currentState = static_cast<CLIMBER_STATE>(GetCurrentState());
    auto targetState = currentState;
    
    if (m_climber != nullptr )
    {
        auto controller = TeleopControl::GetInstance();
        auto isClimbMode  = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::ENABLE_CLIMBER) : false;

        auto isAutoClimb = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::CLIMB_AUTO) : false;

        //auto isClimbManual = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::CLIMBER_STATE_MANUAL) : false;    
        auto isClimbInitialReach = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::CLIMBER_STATE_INITIAL_REACH) : false;
        /*if (isClimbOff)
        {
            targetState = CLIMBER_STATE::OFF;
        }
        else*/ 
        /*else if (isClimbStarting)
        {
            targetState = CLIMBER_STATE::STARTING_CONFIG;
        }*/
        if (isClimbMode)
        {
            if (isAutoClimb)
            {
                //Start at initial climb state by checking if this is the first loop the robot is auto climbing
                if(!m_wasAutoClimb)
                {
                    targetState = CLIMBER_STATE::CLIMB_MID_BAR;
                }
                else if(m_wasAutoClimb)
                {
                    auto currentStatePtr = GetCurrentStatePtr();
                    if (currentStatePtr != nullptr)
                    {
                        auto done = currentStatePtr->AtTarget();
                        if (!done)
                        {
                            targetState = m_prevState;
                        }
                        else if (done && currentState != MAX_STATES)
                        {
                            targetState = static_cast<CLIMBER_STATE>(static_cast<int>(currentState)+1);
                        }
                        m_prevState = targetState;
                    }
                }
                m_wasAutoClimb = true;  
            }
            else
            {
                targetState = CLIMBER_STATE::MANUAL;      
            }  
            if (isClimbInitialReach)
            {
                targetState = CLIMBER_STATE::INITIAL_REACH;
            }          
        }
        else
        {
            m_prevState = CLIMBER_STATE::OFF;
            m_wasAutoClimb = false;
            targetState = CLIMBER_STATE::OFF;
        }

        Logger::GetLogger()->ToNtTable(m_nt, string("state"), targetState);
        if (targetState != currentState)
        {
            Logger::GetLogger()->ToNtTable(m_nt, string("Changing climber State"), targetState);
            SetCurrentState(targetState, true);
        }
    }
}