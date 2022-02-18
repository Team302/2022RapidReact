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

#include <map>
#include <memory>
#include <vector>

// FRC includes
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>

// Team 302 includes
#include <controllers/MechanismTargetData.h>
#include <gamepad/TeleopControl.h>
#include <states/balltransfer/BallTransferState.h>
#include <states/balltransfer/BallTransferStateMgr.h>
#include <states/shooter/ShooterStateMgr.h>
#include <states/IState.h>
#include <states/StateMgr.h>
#include <states/StateStruc.h>
#include <subsys/MechanismFactory.h>
//#include <subsys/MechanismTypes.h>
#include <utils/Logger.h>
#include <xmlmechdata/StateDataDefn.h>


// Third Party Includes

using namespace std;


BallTransferStateMgr* BallTransferStateMgr::m_instance = nullptr;
BallTransferStateMgr* BallTransferStateMgr::GetInstance()
{
	if ( BallTransferStateMgr::m_instance == nullptr )
	{
		BallTransferStateMgr::m_instance = new BallTransferStateMgr();
	}
	return BallTransferStateMgr::m_instance;
}


/// @brief    initialize the state manager, parse the configuration file and create the states.
BallTransferStateMgr::BallTransferStateMgr() : StateMgr(),
                                               m_transfer(MechanismFactory::GetMechanismFactory()->GetBallTransfer()),
                                               m_shooterStateMgr(ShooterStateMgr::GetInstance()),
                                               m_lastManualState(BALL_TRANSFER_STATE::LOAD),
                                               m_nt()
{
    if (m_transfer != nullptr)
    {
        auto ntName = m_transfer->GetNetworkTableName();
        m_nt = nt::NetworkTableInstance::GetDefault().GetTable(ntName);
    }
    else
    {
        m_nt = nt::NetworkTableInstance::GetDefault().GetTable("transfer");
    }

    map<string, StateStruc> stateMap;
    stateMap["BALLTRANSFER_OFF"] = m_offState;
    stateMap["BALLTRANSFER_LOAD"] = m_loadState;
    stateMap["BALLTRANSFER_HOLD"] = m_holdState;
    stateMap["BALLTRANSFER_SHOOT"] = m_feedState;
    Init(m_transfer, stateMap);
}

/// @brief  run the current state
/// @return void
void BallTransferStateMgr::CheckForStateTransition()
{        
    if (m_transfer != nullptr )
    {
        auto isBallPresent = m_transfer->IsBallPresent();
        Logger::GetLogger()->ToNtTable(m_nt, string("Ball Present Sensor"), to_string(isBallPresent));

        // process teleop/manual interrupts
        auto currentState = static_cast<BALL_TRANSFER_STATE>(GetCurrentState());
        auto controller = TeleopControl::GetInstance();
        auto isManualShoot   = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::MANUAL_SHOOT) : false;
        auto isManualKicker  = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::MAN_KICKER) : false;
        auto isAutoShootHigh = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::AUTO_SHOOT_HIGH) : false;
        auto isAutoShootLow  = controller != nullptr ? controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::AUTO_SHOOT_LOW) : false;

        if (isBallPresent && (isAutoShootHigh || isAutoShootLow))
        {
            if (currentState == BALL_TRANSFER_STATE::HOLD)
            {
                Logger::GetLogger()->ToNtTable(m_nt, string("Transfer Ready "), string("true"));
                if (m_shooterStateMgr != nullptr && m_shooterStateMgr->AtTarget())
                {
                    Logger::GetLogger()->ToNtTable(m_nt, string("Shooter Ready "), string("true"));
                    SetCurrentState(BALL_TRANSFER_STATE::FEED, true);
                }
                else // no transition
                {
                    Logger::GetLogger()->ToNtTable(m_nt, string("Shooter Ready "), string("false"));
                }
            }
            else
            {
                Logger::GetLogger()->ToNtTable(m_nt, string("Transfer Ready "), string("false"));
                SetCurrentState(BALL_TRANSFER_STATE::HOLD, true);
            }
        }
        else if (isManualShoot)
        {
            SetCurrentState(BALL_TRANSFER_STATE::FEED, true);
        }
        else if (isManualKicker)
        {
            if (currentState == BALL_TRANSFER_STATE::LOAD)
            {
                SetCurrentState(BALL_TRANSFER_STATE::FEED, true);
            }
            else
            {
                SetCurrentState(BALL_TRANSFER_STATE::LOAD, true);
            }
        }
        else if (isBallPresent)
        {
            SetCurrentState(BALL_TRANSFER_STATE::HOLD, true);
        }
        else 
        {
            SetCurrentState(BALL_TRANSFER_STATE::LOAD, true);
        }
    }
}


