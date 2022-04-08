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


#pragma once

// C++ Includes
#include <map>
#include <vector>

// FRC includes
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>

// Team 302 includes
#include <states/IState.h>
#include <states/StateMgr.h>
#include <states/StateStruc.h>
#include <subsys/Climber.h>

// Third Party Includes

class ClimberStateMgr : public StateMgr
{
    public:
        /// @enum the various states of the impeller
        enum CLIMBER_STATE
        {
            OFF,
            MANUAL,
            STARTING_CONFIG,
            PREP_MID_BAR,
            CLIMB_MID_BAR,
            FRONT_HOOK_PREP_FOR_NEXT_BAR,
            FRONT_HOOK_ROTATE_A,
            FRONT_HOOK_ROTATE_B,
            FRONT_HOOK_ELEVATE,
            FRONT_HOOK_ROTATE_TO_HOOK,
            FRONT_HOOK_LIFT_ROBOT,
            FRONT_HOOK_ROTATE_ARM,
            BACK_HOOK_PREP,
            BACK_HOOK_ROTATE_A,
            BACK_HOOK_LIFT_ROBOT,
            BACK_HOOK_REST
        };

        const std::string m_climberOffXmlString = "OFF";
        const std::string m_climberManualXmlString = "MANUAL";
        const std::string m_climberStartingXmlString = "STARTING_CONFIG";
        const std::string m_climberPrepMidBarXmlString = "PREP_MID_BAR";
        const std::string m_climberClimbMidBarXmlString = "CLIMB_MID_BAR";
        const std::string m_climberFrontHookPrepXmlString = "FRONT_HOOK_PREP_FOR_NEXT_BAR";
        const std::string m_climberFrontHookRotateAXmlString = "FRONT_HOOK_ROTATE_A";
        const std::string m_climberFrontHookRotateBXmlString = "FRONT_HOOK_ROTATE_B";
        const std::string m_climberFrontHookElevateXmlString = "FRONT_HOOK_ELEVATE";
        const std::string m_climberFrontHookRotateToHookXmlString = "FRONT_HOOK_ROTATE_TO_HOOK";
        const std::string m_climberFrontHookLiftRobotXmlString = "FRONT_HOOK_LIFT_ROBOT";
        const std::string m_climberFrontHookRotateArmXmlString = "FRONT_HOOK_ROTATE_ARM";
        const std::string m_climberBackHookPrepXmlString = "BACK_HOOK_PREP";
        const std::string m_climberBackHookRotateAXmlString = "BACK_HOOK_ROTATE_A";
        const std::string m_climberBackHookLiftXmlString = "BACK_HOOK_LIFT_ROBOT";
        const std::string m_climberBackHookRestXmlString = "BACK_HOOK_REST";

        
		/// @brief  Find or create the state manmanager
		/// @return ClimberStateMgr* pointer to the state manager
		static ClimberStateMgr* GetInstance();

        void CheckForStateTransition() override;

    private:
        Climber*                                m_climber;
        std::shared_ptr<nt::NetworkTable>       m_nt;     
        bool                                    m_wasAutoClimb;
        CLIMBER_STATE                           m_prevState;


		static ClimberStateMgr*	m_instance;

        const StateStruc    m_offState = {CLIMBER_STATE::OFF, StateType::CLIMBER, true};
        const StateStruc    m_manualState = {CLIMBER_STATE::MANUAL, StateType::CLIMBER_MANUAL, false};
        const StateStruc    m_startingState = {CLIMBER_STATE::STARTING_CONFIG, StateType::CLIMBER, false};
        const StateStruc    m_prepMidBarState = {CLIMBER_STATE::PREP_MID_BAR, StateType::CLIMBER, false};
        const StateStruc    m_climbMidBarState = {CLIMBER_STATE::CLIMB_MID_BAR, StateType::CLIMBER, false};
        const StateStruc    m_frontHookprepNextBarState = {CLIMBER_STATE::FRONT_HOOK_PREP_FOR_NEXT_BAR, StateType::CLIMBER, false};
        const StateStruc    m_frontHookRotateAState = {CLIMBER_STATE::FRONT_HOOK_ROTATE_A, StateType::CLIMBER, false};
        const StateStruc    m_frontHookRotateBState = {CLIMBER_STATE::FRONT_HOOK_ROTATE_B, StateType::CLIMBER, false};
        const StateStruc    m_frontHookElevateState = {CLIMBER_STATE::FRONT_HOOK_ELEVATE, StateType::CLIMBER, false};
        const StateStruc    m_frontHookRotateToHookState = {CLIMBER_STATE::FRONT_HOOK_ROTATE_TO_HOOK, StateType::CLIMBER, false};
        const StateStruc    m_frontHookLiftState = {CLIMBER_STATE::FRONT_HOOK_LIFT_ROBOT, StateType::CLIMBER, false};
        const StateStruc    m_frontHookRotateArmState = {CLIMBER_STATE::FRONT_HOOK_ROTATE_ARM, StateType::CLIMBER, false};
        const StateStruc    m_backHookPrepState = {CLIMBER_STATE::BACK_HOOK_PREP, StateType::CLIMBER, false};
        const StateStruc    m_backHookRotateAState = {CLIMBER_STATE::BACK_HOOK_ROTATE_A, StateType::CLIMBER, false};
        const StateStruc    m_backHookLiftState = {CLIMBER_STATE::BACK_HOOK_LIFT_ROBOT, StateType::CLIMBER, false};
        const StateStruc    m_backHookRestState = {CLIMBER_STATE::BACK_HOOK_REST, StateType::CLIMBER, false};

        ClimberStateMgr();
        ~ClimberStateMgr() = default;
};