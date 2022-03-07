
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
                                            m_leftIntakeStateMgr(LeftIntakeStateMgr::GetInstance())
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

        if (m_shooterStateMgr != nullptr)
        {
            auto shooterState = static_cast<ShooterStateMgr::SHOOTER_STATE>(m_shooterStateMgr->GetCurrentState());

            if (m_shooter != nullptr)
            {
                auto isAtSpeed = m_shooterStateMgr->AtTarget();
                if (isAtSpeed)
                {
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
                            targetState = INDEXER_STATE::OFF;
                            break;

                        default:
                            targetState = INDEXER_STATE::OFF;
                            break;
                    }
                }
                else
                {
                    targetState = INDEXER_STATE::OFF;
                }
            }
        }
        
        if (m_leftIntakeStateMgr != nullptr)
        {
            auto intakeState = static_cast<IntakeStateMgr::INTAKE_STATE>(m_leftIntakeStateMgr->GetCurrentState());
            targetState = intakeState == IntakeStateMgr::INTAKE_STATE::INTAKE ? INDEXER_STATE::INDEX : INDEXER_STATE::OFF;
        }
        else
        {
            targetState = INDEXER_STATE::OFF;
        }
        if (targetState != currentState)
        {
            SetCurrentState(targetState, true);
        }

    }    
}
