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
    stateMap[m_climberStartingXmlString] = m_startingState;
    stateMap[m_climberPrepMidBarXmlString] = m_prepMidBarState;
    stateMap[m_climberClimbMidBarXmlString] = m_climbMidBarState;
    stateMap[m_climberFrontHookPrepXmlString] = m_frontHookprepNextBarState;
    stateMap[m_climberFrontHookRotateAXmlString] = m_frontHookRotateAState;
    stateMap[m_climberFrontHookRotateBXmlString] = m_frontHookRotateBState;
    stateMap[m_climberFrontHookElevateXmlString] = m_frontHookElevateState;
    stateMap[m_climberFrontHookRotateToHookXmlString] = m_frontHookRotateToHookState;
    stateMap[m_climberFrontHookLiftRobotXmlString] = m_frontHookLiftState;
    stateMap[m_climberFrontHookRotateArmXmlString] = m_frontHookRotateArmState;
    stateMap[m_climberBackHookPrepXmlString] = m_backHookPrepState;
    stateMap[m_climberBackHookRotateAXmlString] = m_backHookRotateAState;
    stateMap[m_climberBackHookLiftXmlString] = m_backHookLiftState;
    stateMap[m_climberBackHookRestXmlString] = m_backHookRestState;

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
        auto isPrepMidbar = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::PREP_MIDBAR_CLIMB) : false;
        auto isAutoClimb = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::CLIMB_AUTO) : false;

        auto isClimbOff = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::CLIMBER_STATE_OFF) : false;
        auto isClimbManual = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::CLIMBER_STATE_MANUAL) : false;
        auto isClimbStarting = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::CLIMBER_STATE_STARTING) : false;
        auto isClimbPrepMid = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::CLIMBER_STATE_PREP_MID) : false;
        auto isClimbMid = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::CLIMBER_STATE_MID) : false;
        auto isClimbFrontPrep = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::CLIMBER_STATE_FRONT_PREP) : false;
        auto isClimbFrontRotateA = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::CLIMBER_STATE_FRONT_ROTATE_A) : false;
        auto isClimbFrontRotateB = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::CLIMBER_STATE_FRONT_ROTATE_B) : false;
        auto isClimbFrontElevate = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::CLIMBER_STATE_FRONT_ELEVATE) : false;
        auto isClimbFrontRotateToHook = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::CLIMBER_STATE_FRONT_ROTATE_TO_HOOK) : false;
        auto isClimbFrontLift = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::CLIMBER_STATE_FRONT_LIFT_ROBOT) : false;
        auto isClimbFrontRotateArm = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::CLIMBER_STATE_ROTATE_ARM) : false;
        auto isClimbBackPrep = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::CLIMBER_STATE_BACK_PREP) : false;
        auto isClimbBackRotate = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::CLIMBER_STATE_BACK_ROTATE_A) : false;
        auto isClimbBackLift = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::CLIMBER_STATE_BACK_LIFT) : false;
        auto isClimbBackRest = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::CLIMBER_STATE_BACK_REST) : false;

        

        /*if (isClimbOff)
        {
            targetState = CLIMBER_STATE::OFF;
        }
        else*/ 
        if (isClimbManual)
        {
            targetState = CLIMBER_STATE::MANUAL;
        }
        /*else if (isClimbStarting)
        {
            targetState = CLIMBER_STATE::STARTING_CONFIG;
        }*/
        else if (isClimbPrepMid)
        {
            targetState = CLIMBER_STATE::PREP_MID_BAR;
        }
        else if (isClimbMid)
        {
            targetState = CLIMBER_STATE::CLIMB_MID_BAR;
        }/*
        else if (isClimbFrontPrep)
        {
            targetState = CLIMBER_STATE::FRONT_HOOK_PREP_FOR_NEXT_BAR;
        }
        else if (isClimbFrontRotateA)
        {
            targetState = CLIMBER_STATE::FRONT_HOOK_ROTATE_A;
        }
        else if (isClimbFrontRotateB)
        {
            targetState = CLIMBER_STATE::FRONT_HOOK_ROTATE_B;
        }
        else if (isClimbFrontElevate)
        {
            targetState = CLIMBER_STATE::FRONT_HOOK_ELEVATE;
        }
        else if (isClimbFrontRotateToHook)
        {
            targetState = CLIMBER_STATE::FRONT_HOOK_ROTATE_TO_HOOK;
        }
        else if (isClimbFrontLift)
        {
            targetState = CLIMBER_STATE::FRONT_HOOK_LIFT_ROBOT;
        }
        else if (isClimbFrontRotateArm)
        {
            targetState = CLIMBER_STATE::FRONT_HOOK_ROTATE_ARM;
        }
        else if (isClimbBackPrep)
        {
            targetState = CLIMBER_STATE::BACK_HOOK_PREP;
        }
        else if (isClimbBackRotate)
        {
            targetState = CLIMBER_STATE::BACK_HOOK_ROTATE_A;
        }
        else if (isClimbBackLift)
        {
            targetState = CLIMBER_STATE::BACK_HOOK_LIFT_ROBOT;
        }
        else if (isClimbBackRest)
        {
            targetState = CLIMBER_STATE::BACK_HOOK_REST;
        }*/

        //if (isClimbMode)
        else if (isClimbMode)
        {
            if (isPrepMidbar)
            {
                targetState = CLIMBER_STATE::PREP_MID_BAR;
                m_prevState = targetState;
            }
            else if (isAutoClimb)
            {
                m_wasAutoClimb = true;
                auto currentStatePtr = GetCurrentStatePtr();
                if (currentStatePtr != nullptr)
                {
                    auto done = currentStatePtr->AtTarget();
                    if (done && currentState != BACK_HOOK_REST)
                    {
                        targetState = static_cast<CLIMBER_STATE>(static_cast<int>(currentState)+1);
                    }
                    m_prevState = targetState;
                }
            }
            else if (m_wasAutoClimb)
            {
                targetState = m_prevState;
            }
            else
            {
                targetState = CLIMBER_STATE::MANUAL;      
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