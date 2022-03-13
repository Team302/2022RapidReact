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
#include <iostream>

// FRC includes
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>

// Team 302 includes
#include <controllers/MechanismTargetData.h>
#include <gamepad/TeleopControl.h>
#include <states/balltransfer/BallTransferState.h>
#include <states/balltransfer/BallTransferStateMgr.h>
#include <states/IState.h>
#include <states/shooter/ShooterStateMgr.h>
#include <states/StateMgr.h>
#include <states/StateStruc.h>
#include <subsys/MechanismFactory.h>
#include <subsys/Shooter.h>
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
                                               m_shooter(MechanismFactory::GetMechanismFactory()->GetShooter()),
                                               m_shooterRPS(0.0),
                                               m_lastManualState(BALL_TRANSFER_STATE::LOAD),
                                               m_lastAutoState(BALL_TRANSFER_STATE::LOAD),
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
    stateMap["BALLTRANSFER_FEED"] = m_feedState;
    stateMap["BALLTRANSFER_SHOOT"] = m_shootState;
    Init(m_transfer, stateMap);
}

/// @brief  run the current state
/// @return void
void BallTransferStateMgr::CheckForStateTransition()
{     
    if (m_transfer != nullptr )
    {
        auto isBallPresent = m_transfer->IsBallPresent();
        auto isLiftForward = m_transfer->IsLiftForward();
        auto shooterRPS = m_shooter != nullptr ? m_shooter->GetSpeed() : 0.0;
        Logger::GetLogger()->ToNtTable(m_nt, string("Ball Present Sensor"), to_string(isBallPresent));
        Logger::GetLogger()->ToNtTable(m_nt, string("Lift Forward Sensor"), to_string(isLiftForward));

        // process teleop/manual interrupts
        auto currentState = static_cast<BALL_TRANSFER_STATE>(GetCurrentState());
        auto targetState = currentState;
        Logger::GetLogger()->ToNtTable(m_nt, string("Current BallTransfer State"), currentState);

        auto isManualShoot      = false;
        auto isManualKicker     = false;
        auto isAutoShootHigh    = false;
        auto isAutoShootLow     = false;
    
        auto controller = TeleopControl::GetInstance();
        if (controller != nullptr)
        {
            isManualShoot   = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::MANUAL_SHOOT);
            isManualKicker  = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::MAN_KICKER);
            isAutoShootHigh = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::AUTO_SHOOT_HIGH);
            isAutoShootLow  = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::AUTO_SHOOT_LOW);
        }

        if (m_shooterStateMgr != nullptr)
        {
            auto shooterState = static_cast<ShooterStateMgr::SHOOTER_STATE>(m_shooterStateMgr->GetCurrentState());
            if (!isManualShoot)
            {
                isManualShoot = shooterState == ShooterStateMgr::SHOOTER_STATE::SHOOT_MANUAL;
            } 
            else if (!isAutoShootHigh)
            {
                isAutoShootHigh = shooterState == ShooterStateMgr::SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_CLOSE ||
                                  shooterState == ShooterStateMgr::SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_FAR;
            } 
            else if (!isAutoShootLow)
            {
                isAutoShootLow = shooterState == ShooterStateMgr::SHOOTER_STATE::SHOOT_LOW_GOAL;
            }

            switch (currentState)
            {
                case BALL_TRANSFER_STATE::OFF:
                    m_shooterRPS = shooterRPS;
                    if (!isBallPresent)
                    {
                        targetState = BALL_TRANSFER_STATE::LOAD;
                    }
                    else if (isManualShoot)
                    {
                        targetState = BALL_TRANSFER_STATE::FEED;
                    }
                    else if ((isAutoShootHigh || isAutoShootLow))
                    {
                        if (m_shooterStateMgr->AtTarget())
                        {
                            targetState = BALL_TRANSFER_STATE::FEED;
                            Logger::GetLogger()->ToNtTable(m_nt, string("Shooter at target "), string("true"));
                        }
                        else
                        {
                            Logger::GetLogger()->ToNtTable(m_nt, string("Shooter at target "), string("false"));
                        }
                    }
                    else if (isBallPresent && !isLiftForward)
                    {
                        targetState = BALL_TRANSFER_STATE::HOLD;
                    }
                    break;

                case BALL_TRANSFER_STATE::LOAD:
                    m_shooterRPS = shooterRPS;
                    if (isBallPresent || isManualKicker)
                    {
                        targetState = BALL_TRANSFER_STATE::HOLD;
                    }
                    break;

                case BALL_TRANSFER_STATE::HOLD:
                    m_shooterRPS = shooterRPS;
                    if (isLiftForward)
                    {
                        targetState = BALL_TRANSFER_STATE::OFF; // don't waste energy
                    }
                    else if (isManualShoot || isManualKicker)
                    {
                        targetState = BALL_TRANSFER_STATE::FEED;
                    }
                    else if ((isAutoShootHigh || isAutoShootLow))
                    {
                        if (m_shooterStateMgr->AtTarget())
                        {
                            targetState = BALL_TRANSFER_STATE::FEED;
                            Logger::GetLogger()->ToNtTable(m_nt, string("Shooter at target "), string("true"));
                        }
                        else
                        {
                            Logger::GetLogger()->ToNtTable(m_nt, string("Shooter at target "), string("false"));
                        }
                    }
                    break;

                case BALL_TRANSFER_STATE::FEED:
                    m_shooterRPS = shooterRPS;
                    if (isManualKicker || isManualShoot || !isBallPresent)
                    {
                        targetState = BALL_TRANSFER_STATE::SHOOT;
                    }
                    break;

                case BALL_TRANSFER_STATE::SHOOT:
                    if (shooterRPS < m_shooterRPS)
                    {
                        targetState = BALL_TRANSFER_STATE::LOAD;
                    }
                    break;

                default:
                    break;
            }

            if (targetState != currentState)
            {
                Logger::GetLogger()->ToNtTable(m_nt, string("Changing BallTransfer State"), targetState);
                SetCurrentState(targetState, true);
            }
        }
    }
}


