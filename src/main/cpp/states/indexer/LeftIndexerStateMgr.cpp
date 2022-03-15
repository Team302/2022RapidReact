
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

// C++ Includes
#include <map>

// FRC includes

// Team 302 includes
#include <states/indexer/LeftIndexerStateMgr.h>
#include <states/StateStruc.h>
#include <subsys/MechanismFactory.h>
#include <subsys/MechanismTypes.h>
#include <gamepad/TeleopControl.h>
#include <states/indexer/RightIndexerStateMgr.h>

#include <utils/Logger.h>

// Third Party Includes

using namespace std;


LeftIndexerStateMgr* LeftIndexerStateMgr::m_instance = nullptr;
LeftIndexerStateMgr* LeftIndexerStateMgr::GetInstance()
{
	if ( LeftIndexerStateMgr::m_instance == nullptr )
	{
		LeftIndexerStateMgr::m_instance = new LeftIndexerStateMgr();
	}
	return LeftIndexerStateMgr::m_instance;
}


/// @brief    initialize the state manager, parse the configuration file and create the states.
LeftIndexerStateMgr::LeftIndexerStateMgr() : IndexerStates(),
                                            m_indexer(MechanismFactory::GetMechanismFactory()->GetLeftIndexer()),
                                            m_shooter(MechanismFactory::GetMechanismFactory()->GetShooter()),
                                            m_shooterStateMgr(ShooterStateMgr::GetInstance()),
                                            m_leftIntakeStateMgr(LeftIntakeStateMgr::GetInstance()),
                                            m_timer(new frc::Timer()),
                                            m_rightIndexerStateMgr(RightIndexerStateMgr::GetInstance())
{
    map<string, StateStruc> stateMap;
    stateMap["INDEXER_OFF"] = m_offState;
    stateMap["INDEXER_INDEX"]  = m_indexState;
    stateMap["INDEXER_EXPEL"] = m_expelState;

    Init(MechanismFactory::GetMechanismFactory()->GetLeftIndexer(), stateMap);
}   


/// @brief  run the current state
/// @return void
void LeftIndexerStateMgr::CheckForStateTransition()
{
    if ( m_indexer != nullptr )
    {
        auto currentState = static_cast<INDEXER_STATE>(GetCurrentState());
        auto targetState = currentState;

        auto controller = TeleopControl::GetInstance();

        if (m_shooterStateMgr != nullptr)
        {
            auto shooterState = static_cast<ShooterStateMgr::SHOOTER_STATE>(m_shooterStateMgr->GetCurrentState());

            if (m_shooter != nullptr)
            {
                Logger::GetLogger()->ToNtTable(string("Indexer Timer"), string("Current time: "), m_timer->Get().to<double>());
                auto isAtSpeed = m_shooterStateMgr->AtTarget();
                if (isAtSpeed)
                {
                    ShooterDelay(); //may stay up here, may not, might change location depending on where m_timer needs to be reset
                    switch (shooterState)
                    {
                        case ShooterStateMgr::SHOOTER_STATE::SHOOT_MANUAL:
                            [[fallthrough]]; //intentional fallthrough

                        case ShooterStateMgr::SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_CLOSE:
                            [[fallthrough]]; //intentional fallthrough

                        case ShooterStateMgr::SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_FAR:
                            [[fallthrough]]; //intentional fallthrough

                        case ShooterStateMgr::SHOOTER_STATE::SHOOT_LOW_GOAL:
                            //ShooterDelay();
                            if (m_delay)
                            {
                                if (m_timer->HasElapsed(units::second_t(0.35)))
                                {
                                    targetState = INDEXER_STATE::INDEX;
                                    if (m_leftIntakeStateMgr != nullptr && controller != nullptr)
                                    {
                                        auto intakeState = static_cast<IntakeStateMgr::INTAKE_STATE>(m_leftIntakeStateMgr->GetCurrentState());
                                        targetState = (intakeState == IntakeStateMgr::INTAKE_STATE::INTAKE && controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::INTAKE_LEFT) ) ? INDEXER_STATE::INDEX : targetState;
                                    }
                                }
                            }
                            else
                            {
                                targetState = INDEXER_STATE::INDEX;
                                /*if (m_leftIntakeStateMgr != nullptr && controller != nullptr)
                                {
                                    auto intakeState = static_cast<IntakeStateMgr::INTAKE_STATE>(m_leftIntakeStateMgr->GetCurrentState());
                                    targetState = (intakeState == IntakeStateMgr::INTAKE_STATE::INTAKE && controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::INTAKE_LEFT) ) ? INDEXER_STATE::INDEX : targetState;
                                } */ 
                            }                            
                            break;
                    
                        case ShooterStateMgr::SHOOTER_STATE::PREPARE_TO_SHOOT:
                            m_timer->Stop();
                            m_timer->Reset();
                            targetState = INDEXER_STATE::OFF;
                            /*if (m_delay)
                            {
                                if (m_timer->HasElapsed(units::second_t(2.0)))
                                {
                                    targetState = INDEXER_STATE::OFF;
                                    if (m_leftIntakeStateMgr != nullptr && controller != nullptr)
                                    {
                                        auto intakeState = static_cast<IntakeStateMgr::INTAKE_STATE>(m_leftIntakeStateMgr->GetCurrentState());
                                        targetState = (intakeState == IntakeStateMgr::INTAKE_STATE::INTAKE && controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::INTAKE_LEFT) ) ? INDEXER_STATE::INDEX : targetState;
                                    }
                                }
                            }
                            else
                            {
                                if (m_leftIntakeStateMgr != nullptr && controller != nullptr)
                                {
                                    auto intakeState = static_cast<IntakeStateMgr::INTAKE_STATE>(m_leftIntakeStateMgr->GetCurrentState());
                                    targetState = (intakeState == IntakeStateMgr::INTAKE_STATE::INTAKE && controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::INTAKE_LEFT) ) ? INDEXER_STATE::INDEX : targetState;
                                } 
                            }*/
                            break;

                        default:
                            targetState = INDEXER_STATE::OFF;
                            //m_timer->Reset();
                            break;
                    }
                }
                else //May need to remove this if we want to index while intaking
                {
                    targetState = INDEXER_STATE::OFF;
                }
            }
        }
        
        if (targetState != currentState)
        {
            SetCurrentState(targetState, true);
        }
    }    
}

void LeftIndexerStateMgr::ShooterDelay()
{
    if (m_rightIndexerStateMgr->GetCurrentState() == RightIndexerStateMgr::INDEXER_STATE::INDEX )
    {
        m_timer->Start();
        m_delay = true;
    }
    else
    {
        m_delay = false;
        m_timer->Stop();
        m_timer->Reset();
    }
}
