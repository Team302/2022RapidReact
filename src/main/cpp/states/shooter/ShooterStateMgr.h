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

// FRC includes
#include <networktables/NetworkTable.h>
#include <frc/Timer.h>

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
            PREPARE_TO_SHOOT
        };
        const std::string m_shooterOffXmlString = "SHOOTER_OFF";
        const std::string m_shooterHighGoalCloseXmlString = "SHOOT_HIGHGOAL_CLOSE";
        const std::string m_shooterHighGoalFarXmlString = "SHOOT_HIGHGOAL_FAR";
        const std::string m_shooterLowGoalXmlString = "SHOOT_LOWGOAL";
        const std::string m_shooterManualXmlString = "MANUAL_SHOOT";
        const std::string m_shooterPrepareXmlString = "PREPARETOSHOOT";
        
        const std::map<const std::string, SHOOTER_STATE> m_shooterXmlStringToStateEnumMap
        {   {m_shooterOffXmlString, SHOOTER_STATE::OFF},
            {m_shooterHighGoalCloseXmlString, SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_CLOSE},
            {m_shooterHighGoalFarXmlString, SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_FAR},
            {m_shooterLowGoalXmlString, SHOOTER_STATE::SHOOT_LOW_GOAL},
            {m_shooterManualXmlString, SHOOTER_STATE::SHOOT_MANUAL},
            {m_shooterPrepareXmlString, SHOOTER_STATE::PREPARE_TO_SHOOT}
        };

        
		/// @brief  Find or create the state manmanager
		/// @return IntakeStateMgr* pointer to the state manager
		static ShooterStateMgr* GetInstance();

        void CheckForStateTransition() override;
        bool AtTarget() const;
        bool IsShooting() const;
    private:

        ShooterStateMgr();
        ~ShooterStateMgr() = default;
        
        DragonLimelight* m_dragonLimeLight;
        Shooter*                                m_shooter;
        std::shared_ptr<nt::NetworkTable>       m_nt;
        bool m_buttonTriggerStateChange;


        const double m_CHANGE_STATE_TARGET = 120.0; 
		static ShooterStateMgr*	m_instance;
        const StateStruc m_offState = {SHOOTER_STATE::OFF, StateType::SHOOTER, true};
        const StateStruc m_shootFarState = {SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_FAR, StateType::SHOOTER_AUTO, false};
        const StateStruc m_shootCloseState = {SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_CLOSE, StateType::SHOOTER_AUTO, false};
        const StateStruc m_shootLowState = {SHOOTER_STATE::SHOOT_LOW_GOAL, StateType::SHOOTER, false};
        const StateStruc m_manualShootState = {SHOOTER_STATE::SHOOT_MANUAL, StateType::SHOOTER, false};
        const StateStruc m_prepareToShoot = {SHOOTER_STATE::PREPARE_TO_SHOOT, StateType::SHOOTER, false};
};