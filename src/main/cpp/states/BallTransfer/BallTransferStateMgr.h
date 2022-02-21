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
#include <networktables/NetworkTable.h>

// Team 302 includes
#include <states/shooter/ShooterStateMgr.h>
#include <states/StateMgr.h>
#include <states/StateStruc.h>
#include <subsys/BallTransfer.h>


// Third Party Includes

class BallTransferStateMgr : public StateMgr
{
    public:
        /// @enum the various states of the BallTransfer
        enum BALL_TRANSFER_STATE
        {
            OFF,
            LOAD,
            HOLD,
            FEED,
            MAX_BALL_TRANSFER_STATES
        };

        
		/// @brief  Find or create the state manmanager
		/// @return BallTransferStateMgr* pointer to the state manager
		static BallTransferStateMgr* GetInstance();
        void CheckForStateTransition() override;


    private:
        BallTransfer*                           m_transfer;
        ShooterStateMgr*                        m_shooterStateMgr;
        BALL_TRANSFER_STATE                     m_lastManualState;
        std::shared_ptr<nt::NetworkTable>       m_nt;     



        BallTransferStateMgr();
        ~BallTransferStateMgr() = default;

		static BallTransferStateMgr*	m_instance;

        const StateStruc  m_offState = {BALL_TRANSFER_STATE::OFF, StateType::BALL_TRANSFER, true};
        const StateStruc  m_loadState = {BALL_TRANSFER_STATE::LOAD, StateType::BALL_TRANSFER, false};
        const StateStruc  m_holdState = {BALL_TRANSFER_STATE::HOLD, StateType::BALL_TRANSFER, false};
        const StateStruc  m_feedState = {BALL_TRANSFER_STATE::FEED, StateType::BALL_TRANSFER, false};
};
