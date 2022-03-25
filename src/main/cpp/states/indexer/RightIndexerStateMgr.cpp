
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
#include <states/indexer/RightIndexerStateMgr.h>
#include <states/StateStruc.h>
#include <subsys/MechanismFactory.h>
#include <subsys/MechanismTypes.h>
#include <gamepad/TeleopControl.h>

#include <utils/Logger.h>

// Third Party Includes

using namespace std;


RightIndexerStateMgr* RightIndexerStateMgr::m_instance = nullptr;
RightIndexerStateMgr* RightIndexerStateMgr::GetInstance()
{
	if ( RightIndexerStateMgr::m_instance == nullptr )
	{
	    auto mechFactory = MechanismFactory::GetMechanismFactory();
	    auto indexer = mechFactory->GetRightIndexer();
        if (indexer != nullptr)
        {
    		RightIndexerStateMgr::m_instance = new RightIndexerStateMgr();
        }
	}
	return RightIndexerStateMgr::m_instance;
}


/// @brief    initialize the state manager, parse the configuration file and create the states.
RightIndexerStateMgr::RightIndexerStateMgr() : IndexerStates(),
                                                m_indexer(MechanismFactory::GetMechanismFactory()->GetRightIndexer()),
                                                m_shooterStateMgr(ShooterStateMgr::GetInstance()),
                                                m_shooter(MechanismFactory::GetMechanismFactory()->GetShooter()),
                                                m_rightIntakeStateMgr(RightIntakeStateMgr::GetInstance()),
                                                m_timer(new frc::Timer())

{
    map<string, StateStruc> stateMap;
    stateMap["INDEXER_OFF"] = m_offState;
    stateMap["INDEXER_INDEX"]  = m_indexState;
    stateMap["INDEXER_EXPEL"] = m_expelState;

    Init(MechanismFactory::GetMechanismFactory()->GetRightIndexer(), stateMap);
}   


/// @brief  run the current state
/// @return void
void RightIndexerStateMgr::CheckForStateTransition()
{
    if ( m_indexer != nullptr )
    {
        auto currentState = static_cast<INDEXER_STATE>(GetCurrentState());
        auto targetState = currentState;

        //auto controller = TeleopControl::GetInstance();

       // bool ballPresent = m_indexer->IsBallPresent();
        
        if (m_shooterStateMgr != nullptr)
        {
            auto shooterState = static_cast<ShooterStateMgr::SHOOTER_STATE>(m_shooterStateMgr->GetCurrentState());
            if (m_shooter != nullptr)
            {
                auto isAtSpeed = m_shooterStateMgr->AtTarget();
                if (isAtSpeed)
                {
                    Logger::GetLogger()->ToNtTable(string("Indexer Timer"), string("Current time left: "), m_timer->Get().to<double>());
                    DelayForLeftIndexer();
                    switch (shooterState)
                    {
                        case ShooterStateMgr::SHOOTER_STATE::SHOOT_MANUAL:
                            [[fallthrough]]; //intentional fallthrough

                        case ShooterStateMgr::SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_CLOSE:
                            [[fallthrough]]; //intentional fallthrough

                        case ShooterStateMgr::SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_FAR:
                            [[fallthrough]]; //intentional fallthrough

                        case ShooterStateMgr::SHOOTER_STATE::SHOOT_LOW_GOAL:
                            targetState = INDEXER_STATE::INDEX;
                            break;

                        case ShooterStateMgr::SHOOTER_STATE::PREPARE_TO_SHOOT:
                            m_timer->Stop();
                            m_timer->Reset();
                            m_delay = false;
                            targetState = INDEXER_STATE::OFF;
                            break;

                        default:
                            targetState = INDEXER_STATE::OFF;
                            break;
                    }
                }
                else //May need to remove this if we want to index while intaking
                {
                    targetState = INDEXER_STATE::OFF;
                }
            }
        }

        //if (m_rightIntakeStateMgr != nullptr && controller != nullptr && !ballPresent) 
        /*
        if (m_rightIntakeStateMgr != nullptr && controller != nullptr) 
        { 
            auto intakeState = static_cast<IntakeStateMgr::INTAKE_STATE>(m_rightIntakeStateMgr->GetCurrentState()); 
            targetState = (intakeState == IntakeStateMgr::INTAKE_STATE::INTAKE && controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::INTAKE_RIGHT) ) ? INDEXER_STATE::INDEX : targetState; 
        } */

        if (m_timer->HasElapsed(units::second_t(1.2)))
        {
           m_timer->Reset();
        }

        if (m_delay)
        {
            if (m_timer->HasElapsed(units::second_t(0.6)))
            {
                targetState = INDEXER_STATE::OFF;
            }
        }
        
        if (targetState != currentState)
        {
            SetCurrentState(targetState, true);
        }
    }    
}

void RightIndexerStateMgr::DelayForLeftIndexer()
{
    if (GetCurrentState() == INDEXER_STATE::INDEX )
    {
        m_timer->Start();
        m_delay = true;
    }
}