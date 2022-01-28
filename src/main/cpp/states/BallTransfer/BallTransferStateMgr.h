#pragma once

// C++ Includes

// FRC includes

// Team 302 includes
#include <states/StateMgr.h>
#include <states/StateStruc.h>


// Third Party Includes

class BallTransferStateMgr : public StateMgr
{
    public:
        /// @enum the various states of the BallTransfer
        enum BALL_TRANSFER_STATE
        {
            OFF,
            INTAKE,
            LIFT,
            MAX_BALL_TRANSFER_STATES
        };

        
		/// @brief  Find or create the state manmanager
		/// @return BallTransferStateMgr* pointer to the state manager
		static BallTransferStateMgr* GetInstance();
        void CheckForStateTransition() override;


    private:
        BallTransferStateMgr();
        ~BallTransferStateMgr() = default;

		static BallTransferStateMgr*	m_instance;

        const StateStruc  m_offState = {BALL_TRANSFER_STATE::OFF, StateType::BALLTRANSER, true};
        const StateStruc  m_intakeState = {BALL_TRANSFER_STATE::INTAKE, StateType::BALLTRANSER, false};
        const StateStruc  m_liftState = {BALL_TRANSFER_STATE::LIFT, StateType::BALLTRANSER, false};
};
