#pragma once

// C++ Includes

// FRC includes

// Team 302 includes
#include <states/StateMgr.h>
#include <states/StateStruc.h>



// Third Party Includes

class ShooterStateMgr : public StateMgr
{
    public:
        /// @enum the various states of the Intake
        enum SHOOTER_STATE
        {
            OFF,
            ON
        };

        
		/// @brief  Find or create the state manmanager
		/// @return IntakeStateMgr* pointer to the state manager
		static ShooterStateMgr* GetInstance();
        void CheckForStateTransition() override;

    private:

        ShooterStateMgr();
        ~ShooterStateMgr() = default;

		static ShooterStateMgr*	m_instance;
    
        const StateStruc  m_offState = {SHOOTER_STATE::OFF, StateType::SHOOTER, true};
        const StateStruc  m_onState = {SHOOTER_STATE::ON, StateType::SHOOTER, false};
};