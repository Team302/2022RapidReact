#pragma once

// C++ Includes

// FRC includes

// Team 302 includes
#include <states/StateMgr.h>
#include <states/StateStruc.h>



// Third Party Includes

class ServoStateMgr : public StateMgr
{
    public:
        /// @enum the various states of the Intake
        enum SERVO_STATE
        {
            LOOK_RIGHT,
            LOOK_LEFT,
            MAX_INTAKE_STATES
        };

        
		/// @brief  Find or create the state manmanager
		/// @return IntakeStateMgr* pointer to the state manager
		static ServoStateMgr* GetInstance();
        void CheckForStateTransition() override;

    private:

        ServoStateMgr();
        ~ServoStateMgr() = default;

		static ServoStateMgr*	m_instance;

        const StateStruc  m_rightState = {SERVO_STATE::LOOK_RIGHT, StateType::SERVO, true};
        const StateStruc  m_leftState = {SERVO_STATE::LOOK_LEFT, StateType::SERVO, false};
};