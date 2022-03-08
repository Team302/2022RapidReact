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
#include <string>
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
            INITIAL_REACH,
            RETRACT,
            RELEASE,
            REACH_TO_BAR,
            ROTATE_OUT,
            ROTATE_IN,
            HOLD,
            MAX_CLIMBER_STATES
        };

        
		/// @brief  Find or create the state manmanager
		/// @return ClimberStateMgr* pointer to the state manager
		static ClimberStateMgr* GetInstance();

        void CheckForStateTransition() override;

    private:
        Climber*                                m_climber;
        std::shared_ptr<nt::NetworkTable>       m_nt;     


		static ClimberStateMgr*	m_instance;

        const StateStruc    m_climberOffState = {CLIMBER_STATE::OFF, StateType::CLIMBER, true};
        const StateStruc    m_climberManualState = {CLIMBER_STATE::MANUAL, StateType::CLIMBER, false};
        const StateStruc    m_climberInitialReachState = {CLIMBER_STATE::INITIAL_REACH, StateType::CLIMBER, false};
        const StateStruc    m_climberRetractState = {CLIMBER_STATE::RETRACT, StateType::CLIMBER, false};
        const StateStruc    m_climberReleaseState = {CLIMBER_STATE::RELEASE, StateType::CLIMBER, false};
        const StateStruc    m_climberReachToBarState = {CLIMBER_STATE::REACH_TO_BAR, StateType::CLIMBER, false};
        const StateStruc    m_climberRotateOutState = {CLIMBER_STATE::ROTATE_OUT, StateType::CLIMBER, false};
        const StateStruc    m_climberRotateInState = {CLIMBER_STATE::ROTATE_IN, StateType::CLIMBER, false};
        const StateStruc    m_climberHoldState = {CLIMBER_STATE::HOLD, StateType::CLIMBER, false};


        const std::map<std::string, StateStruc> m_climberStateMap
        {
            {std::string("CLIMBEROFF"), m_climberOffState},
            {std::string("CLIMBERMANUAL"), m_climberManualState},
            {std::string("CLIMBERINITIALREACH"), m_climberInitialReachState},
            {std::string("CLIMBERRETRACT"), m_climberRetractState},
            {std::string("CLIMBERRELEASE"), m_climberReleaseState},
            {std::string("CLIMBERREACHTOBAR"), m_climberReachToBarState},
            {std::string("CLIMBERROTATEOUT"), m_climberRotateOutState},
            {std::string("CLIMBERROTATEIN"), m_climberRotateInState},
            {std::string("CLIMBERHOLD"), m_climberHoldState}
        };
        ClimberStateMgr();
        ~ClimberStateMgr() = default;
};