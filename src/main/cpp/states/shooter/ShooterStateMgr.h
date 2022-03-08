//====================================================================================================================================================
/// Copyright 2022 Lake Orion Robotics FIRST Team 302 
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#pragma once

// C++ Includes
#include <map>
#include <string>

// FRC includes
#include <networktables/NetworkTable.h>

// Team 302 includes
#include <states/StateMgr.h>
#include <states/StateStruc.h>
#include <hw/DragonLimelight.h>
#include <subsys/Shooter.h>




// Third Party Includes

class ShooterStateMgr : public StateMgr
{
    public:
        /// @enum the various states of the Intake
        enum SHOOTER_STATE
        {
            OFF,
            AUTO_SHOOT_HIGH_GOAL_FAR,
            AUTO_SHOOT_HIGH_GOAL_CLOSE,
            SHOOT_LOW_GOAL,
            SHOOT_MANUAL,
            SHOOTER_HOOD_ADJUST,
            PREPARE_TO_SHOOT
        };

        // these are the strings in the XML files
        const std::string SHOOTER_STATE_OFF_STRING = std::string("SHOOTER_OFF");
        const std::string SHOOTER_STATE_HIGH_FAR_STRING = std::string("SHOOT_HIGHGOAL_CLOSE");
        const std::string SHOOTER_STATE_HIGH_CLOSE_STRING = std::string("SHOOT_HIGHGOAL_FAR");
        const std::string SHOOTER_STATE_LOW_STRING = std::string("SHOOT_LOWGOAL");
        const std::string SHOOTER_STATE_MANUAL_STRING = std::string("MANUAL_SHOOT");
        const std::string SHOOTER_STATE_HOOD_STRING = std::string("ADJUSTHOOD");
        const std::string SHOOTER_STATE_PREPARE_STRING = std::string("PREPARETOSHOOT");

        const std::map<std::string, SHOOTER_STATE> m_intakeStringEnumMap
        {
            {SHOOTER_STATE_OFF_STRING, SHOOTER_STATE::OFF},
            {SHOOTER_STATE_HIGH_FAR_STRING, SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_FAR},
            {SHOOTER_STATE_HIGH_CLOSE_STRING, SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_CLOSE},
            {SHOOTER_STATE_LOW_STRING, SHOOTER_STATE::SHOOT_LOW_GOAL},
            {SHOOTER_STATE_MANUAL_STRING, SHOOTER_STATE::SHOOT_MANUAL},
            {SHOOTER_STATE_HOOD_STRING, SHOOTER_STATE::SHOOTER_HOOD_ADJUST},
            {SHOOTER_STATE_PREPARE_STRING, SHOOTER_STATE::PREPARE_TO_SHOOT}
        };
        
		/// @brief  Find or create the state manmanager
		/// @return IntakeStateMgr* pointer to the state manager
		static ShooterStateMgr* GetInstance();
        void CheckForStateTransition() override;
        bool AtTarget() const;
    private:

        ShooterStateMgr();
        ~ShooterStateMgr() = default;
        
        DragonLimelight* m_dragonLimeLight;
        Shooter*                                m_shooter;
        std::shared_ptr<nt::NetworkTable>       m_nt;     

        const StateStruc m_shooterOffState = {SHOOTER_STATE::OFF, StateType::SHOOTER, true};
        const StateStruc m_shooterShootFarState = {SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_FAR, StateType::SHOOTER_AUTO, false};
        const StateStruc m_shooterShootCloseState = {SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_CLOSE, StateType::SHOOTER_AUTO, false};
        const StateStruc m_shooterShootLowState = {SHOOTER_STATE::SHOOT_LOW_GOAL, StateType::SHOOTER, false};
        const StateStruc m_shooterManualShootState = {SHOOTER_STATE::SHOOT_MANUAL, StateType::SHOOTER, false};
        const StateStruc m_shooterHoodAdjust = {SHOOTER_STATE::SHOOTER_HOOD_ADJUST, StateType::SHOOTER_MANUAL, false};
        const StateStruc m_shooterPrepareToShoot = {SHOOTER_STATE::PREPARE_TO_SHOOT, StateType::SHOOTER, false};

        const std::map<std::string, StateStruc> m_shooterStateMap
        {
            {SHOOTER_STATE_OFF_STRING, m_shooterOffState},
            {SHOOTER_STATE_HIGH_CLOSE_STRING, m_shooterShootCloseState},
            {SHOOTER_STATE_HIGH_FAR_STRING, m_shooterShootFarState},
            {SHOOTER_STATE_LOW_STRING, m_shooterShootLowState},
            {SHOOTER_STATE_MANUAL_STRING, m_shooterManualShootState},
            {SHOOTER_STATE_HOOD_STRING, m_shooterHoodAdjust},
            {SHOOTER_STATE_PREPARE_STRING, m_shooterPrepareToShoot}
        };

        const double m_CHANGE_STATE_TARGET = 120.0; 
		static ShooterStateMgr*	m_instance;
};