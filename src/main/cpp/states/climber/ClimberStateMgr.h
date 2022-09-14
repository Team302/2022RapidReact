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
#include <frc/Timer.h>

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
            UNINITIALIZED,
            MANUAL,
            ZERO_BEFORE_CLIMB,
            INITIAL_REACH,
            CLIMB_MID_BAR,
            PREPARE_EXTEND_MID_BAR,
            EXTEND_MID_BAR,
            ROTATE_MID_BAR,
            REACH_HIGH_BAR,
            CLIMB_HIGH_BAR,
            PREPARE_EXTEND_HIGH_BAR,
            EXTEND_HIGH_BAR,
            CLIMB_TRAVERSAL_BAR,
            MAX_STATES
        };

        const std::string m_climberOffXmlString = "CLIMBER_OFF";
        const std::string m_climberUninitializedXmlString = "CLIMBER_OFF";
        const std::string m_climberManualXmlString = "CLIMBER_MANUAL";
        const std::string m_climberZeroClimbString = "CLIMBER_ZERO_BEFORE_CLIMB";
        const std::string m_climberInitialReachXmlString = "CLIMBER_INITIALREACH";
        const std::string m_climberClimbMidXmlString = "CLIMBER_CLIMB_MID_BAR";
        const std::string m_climberExtendMidXmlString = "CLIMBER_EXTEND_MID_BAR";
        const std::string m_climberPrepareExtendMidXmlString = "CLIMBER_PREPARE_EXTEND_MID_BAR";
        const std::string m_climberRotateMidXmlString = "CLIMBER_ROTATE_MID_BAR";
        const std::string m_climberReachHighXmlString = "CLIMBER_REACH_HIGH_BAR";
        const std::string m_climberClimbHighXmlString = "CLIMBER_CLIMB_HIGH_BAR";
        const std::string m_climberPrepareExtendHighXmlString = "CLIMBER_PREPARE_EXTEND_HIGH_BAR";
        const std::string m_climberExtendHighXmlString = "CLIMBER_EXTEND_HIGH_BAR";
        const std::string m_climberClimbTraversalXmlString = "CLIMBER_CLIMB_TRAVERSAL_BAR";
        
		/// @brief  Find or create the state manmanager
		/// @return ClimberStateMgr* pointer to the state manager
		static ClimberStateMgr* GetInstance();

        void CheckForStateTransition() override;

    private:
        /// @brief Check to see if driver is trying to climb manually
        /// @return Bool - if there is input or not
        bool CheckForManualInput();

        Climber*                                m_climber;
        std::shared_ptr<nt::NetworkTable>       m_nt;     
        bool                                    m_wasAutoClimb;
        CLIMBER_STATE                           m_prevState;
        bool                                    m_hasZeroed;
        CLIMBER_STATE                           m_currentAutoState;

        frc::Timer                              m_autoTimer;


		static ClimberStateMgr*	m_instance;

        const StateStruc    m_offState = {CLIMBER_STATE::OFF, StateType::CLIMBER, false};
        const StateStruc    m_uninitializedState = {CLIMBER_STATE::UNINITIALIZED, StateType::CLIMBER, true};
        const StateStruc    m_manualState = {CLIMBER_STATE::MANUAL, StateType::CLIMBER_MANUAL, false};
        const StateStruc    m_zeroClimbState = {CLIMBER_STATE::ZERO_BEFORE_CLIMB, StateType::CLIMBER, false};
        const StateStruc    m_initialReachState = {CLIMBER_STATE::INITIAL_REACH, StateType::CLIMBER, false};
        const StateStruc    m_climbMidState = {CLIMBER_STATE::CLIMB_MID_BAR, StateType::CLIMBER, false};
        const StateStruc    m_prepareExtendMidState = {CLIMBER_STATE::PREPARE_EXTEND_MID_BAR, StateType::CLIMBER, false};
        const StateStruc    m_extendMidState = {CLIMBER_STATE::EXTEND_MID_BAR, StateType::CLIMBER, false};
        const StateStruc    m_rotateMidState = {CLIMBER_STATE::ROTATE_MID_BAR, StateType::CLIMBER, false};
        const StateStruc    m_reachHighState = {CLIMBER_STATE::REACH_HIGH_BAR, StateType::CLIMBER, false};
        const StateStruc    m_climbHighState = {CLIMBER_STATE::CLIMB_HIGH_BAR, StateType::CLIMBER, false};
        const StateStruc    m_prepareExtendHighState = {CLIMBER_STATE::PREPARE_EXTEND_HIGH_BAR, StateType::CLIMBER, false};
        const StateStruc    m_extendHighState = {CLIMBER_STATE::EXTEND_HIGH_BAR, StateType::CLIMBER, false};
        const StateStruc    m_climbTraversalState = {CLIMBER_STATE::CLIMB_TRAVERSAL_BAR, StateType::CLIMBER, false};

        ClimberStateMgr();
        ~ClimberStateMgr() = default;
};