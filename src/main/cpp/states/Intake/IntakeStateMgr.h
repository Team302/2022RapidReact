
#pragma once

// C++ Includes

// FRC includes

// Team 302 includes
#include <states/StateMgr.h>
#include <states/StateStruc.h>



// Third Party Includes

class IntakeStateMgr : public StateMgr
{
    public:
        /// @enum the various states of the Intake
        enum INTAKE_STATE
        {
           // OFF,
            INTAKE,
           // EXPEL,
           // MAX_INTAKE_STATES
        };

        
		/// @brief  Find or create the state manmanager
		/// @return IntakeStateMgr* pointer to the state manager
		static IntakeStateMgr* GetInstance();
        void CheckForStateTransition() override;

    private:

        IntakeStateMgr();
        ~IntakeStateMgr() = default;

		static IntakeStateMgr*	m_instance;

      //  const StateStruc  m_offState = {INTAKE_STATE::OFF, StateType::INTAKE, true};
        const StateStruc  m_intakeState = {INTAKE_STATE::INTAKE, StateType::INTAKE, false};
      //  const StateStruc  m_expelState = {INTAKE_STATE::EXPEL, StateType::INTAKE, false};
};