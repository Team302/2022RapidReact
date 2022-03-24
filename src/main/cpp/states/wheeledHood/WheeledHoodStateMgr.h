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
#include <memory>

// FRC includes
#include <networktables/NetworkTable.h>

// Team 302 includes
#include <states/shooter/ShooterStateMgr.h>
#include <states/StateMgr.h>
#include <states/StateStruc.h>

// Third Party Includes

class WheeledHoodStateMgr : public StateMgr
{
    public:
		/// @brief  Find or create the state manmanager
		/// @return WheeledHoodStateMgr* pointer to the state manager
		static WheeledHoodStateMgr* GetInstance();

        void CheckForStateTransition() override;
    private:

        WheeledHoodStateMgr();
        ~WheeledHoodStateMgr() = default;
        
        std::shared_ptr<nt::NetworkTable>       m_nt;

		static WheeledHoodStateMgr*	m_instance;
        const StateStruc m_offState = {ShooterStateMgr::SHOOTER_STATE::OFF, StateType::WHEELED_HOOD, true};
        const StateStruc m_shootFarState = {ShooterStateMgr::SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_FAR, StateType::WHEELED_HOOD, false};
        const StateStruc m_shootCloseState = {ShooterStateMgr::SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_CLOSE, StateType::WHEELED_HOOD, false};
        const StateStruc m_shootLowState = {ShooterStateMgr::SHOOTER_STATE::SHOOT_LOW_GOAL, StateType::WHEELED_HOOD, false};
        const StateStruc m_manualShootState = {ShooterStateMgr::SHOOTER_STATE::SHOOT_MANUAL, StateType::WHEELED_HOOD, false};
        const StateStruc m_prepareToShoot = {ShooterStateMgr::SHOOTER_STATE::PREPARE_TO_SHOOT, StateType::WHEELED_HOOD, false};
};