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
#include <string>

// FRC includes
#include <networktables/NetworkTable.h>

// Team 302 includes
#include <states/shooter/ShooterStateMgr.h>
#include <states/StateMgr.h>
#include <states/StateStruc.h>
#include <subsys/BallTransfer.h>
#include <subsys/Shooter.h>


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
            SHOOT,
            MAX_BALL_TRANSFER_STATES
        };

        // these are the strings in the XML files
        const std::string BALL_TRANSFER_STATE_OFF_STRING = std::string("BALLTRANSFER_OFF");
        const std::string BALL_TRANSFER_STATE_LOAD_STRING = std::string("BALLTRANSFER_LOAD");
        const std::string BALL_TRANSFER_STATE_HOLD_STRING = std::string("BALLTRANSFER_HOLD");
        const std::string BALL_TRANSFER_STATE_FEED_STRING = std::string("BALLTRANSFER_FEED");
        const std::string BALL_TRANSFER_STATE_SHOOT_STRING = std::string("BALLTRANSFER_SHOOT");

        const std::map<std::string, BALL_TRANSFER_STATE> m_ballTransferStringEnumMap
        {
            {BALL_TRANSFER_STATE_OFF_STRING, BALL_TRANSFER_STATE::OFF},
            {BALL_TRANSFER_STATE_LOAD_STRING, BALL_TRANSFER_STATE::LOAD},
            {BALL_TRANSFER_STATE_HOLD_STRING, BALL_TRANSFER_STATE::HOLD},
            {BALL_TRANSFER_STATE_FEED_STRING, BALL_TRANSFER_STATE::FEED},
            {BALL_TRANSFER_STATE_SHOOT_STRING, BALL_TRANSFER_STATE::SHOOT}
        };
        
		/// @brief  Find or create the state manmanager
		/// @return BallTransferStateMgr* pointer to the state manager
		static BallTransferStateMgr* GetInstance();
        void CheckForStateTransition() override;


    private:
        BallTransfer*                           m_transfer;
        ShooterStateMgr*                        m_shooterStateMgr;
        Shooter*                                m_shooter;
        double                                  m_shooterRPS;
        BALL_TRANSFER_STATE                     m_lastManualState;
        BALL_TRANSFER_STATE                     m_lastAutoState;
        std::shared_ptr<nt::NetworkTable>       m_nt;     


        const StateStruc  m_balltransferOffState = {BALL_TRANSFER_STATE::OFF, StateType::BALL_TRANSFER, true};
        const StateStruc  m_balltransferLoadState = {BALL_TRANSFER_STATE::LOAD, StateType::BALL_TRANSFER, false};
        const StateStruc  m_balltransferHoldState = {BALL_TRANSFER_STATE::HOLD, StateType::BALL_TRANSFER, false};
        const StateStruc  m_balltransferFeedState = {BALL_TRANSFER_STATE::FEED, StateType::BALL_TRANSFER, false};
        const StateStruc  m_balltransferShootState = {BALL_TRANSFER_STATE::SHOOT, StateType::BALL_TRANSFER, false};

        const std::map<std::string, StateStruc> m_ballTransferStateMap
        {
            {BALL_TRANSFER_STATE_OFF_STRING, m_balltransferOffState},
            {BALL_TRANSFER_STATE_LOAD_STRING, m_balltransferLoadState},
            {BALL_TRANSFER_STATE_HOLD_STRING, m_balltransferHoldState},
            {BALL_TRANSFER_STATE_FEED_STRING, m_balltransferFeedState},
            {BALL_TRANSFER_STATE_SHOOT_STRING, m_balltransferShootState}
        };

        BallTransferStateMgr();
        ~BallTransferStateMgr() = default;

		static BallTransferStateMgr*	m_instance;
};
