
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

// FRC includes

// Team 302 includes
#include <states/StateStruc.h>
#include <states/shooter/ShooterStateMgr.h>
#include <states/StateMgr.h>

#include <subsys/Shooter.h>
#include <subsys/Lift.h>


// Third Party Includes

class LiftStateMgr : public StateMgr
{
    public:
        /// @enum the various states of the Lift
        enum LIFT_STATE
        {
            OFF,
            LIFT,
            LOWER,
            MAX_BALL_TRANSFER_STATES
        };

		/// @brief  Find or create the state manmanager
		/// @return RightIntakeStateMgr* pointer to the state manager
		static LiftStateMgr* GetInstance();
        void CheckForStateTransition() override;

    protected:
        const StateStruc  m_offState = {LIFT_STATE::OFF, StateType::LIFT, true};
        const StateStruc  m_liftState = {LIFT_STATE::LIFT, StateType::LIFT, false};
        const StateStruc  m_lowerState = {LIFT_STATE::LOWER, StateType::LIFT, false};


    private:
        LiftStateMgr();
        ~LiftStateMgr() = default;

		static LiftStateMgr*	m_instance;
        ShooterStateMgr*        m_shooterStateMgr;
        Shooter*                m_shooter;
        Lift*                   m_lift;

};
