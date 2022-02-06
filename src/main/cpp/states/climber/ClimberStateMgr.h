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

// Third Party Includes

class ClimberStateMgr : public StateMgr
{
    public:
        /// @enum the various states of the impeller
        enum CLIMBER_STATE
        {
            OFF,
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

		static ClimberStateMgr*	m_instance;

        const StateStruc    m_offState = {CLIMBER_STATE::OFF, StateType::CLIMBER, true};
        const StateStruc    m_initialReachState = {CLIMBER_STATE::INITIAL_REACH, StateType::CLIMBER, false};
        const StateStruc    m_retractState = {CLIMBER_STATE::RETRACT, StateType::CLIMBER, false};
        const StateStruc    m_releaseState = {CLIMBER_STATE::RELEASE, StateType::CLIMBER, false};
        const StateStruc    m_reachToBarState = {CLIMBER_STATE::REACH_TO_BAR, StateType::CLIMBER, false};
        const StateStruc    m_rotateOutState = {CLIMBER_STATE::ROTATE_OUT, StateType::CLIMBER, false};
        const StateStruc    m_rotateInState = {CLIMBER_STATE::ROTATE_IN, StateType::CLIMBER, false};
        const StateStruc    m_holdState = {CLIMBER_STATE::HOLD, StateType::CLIMBER, false};

        ClimberStateMgr();
        ~ClimberStateMgr() = default;
};