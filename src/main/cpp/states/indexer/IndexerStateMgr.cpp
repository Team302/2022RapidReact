
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
#include <iostream>

// FRC includes

// Team 302 includes
#include <gamepad/TeleopControl.h>
#include <states/indexer/IndexerStateMgr.h>
#include <states/Intake/IntakeStateMgr.h>
#include <states/Intake/LeftIntakeStateMgr.h>
#include <states/Intake/RightIntakeStateMgr.h>
#include <states/StateMgr.h>
#include <states/StateStruc.h>
#include <subsys/MechanismFactory.h>
#include <subsys/MechanismTypes.h>
#include <utils/Logger.h>

// Third Party Includes

using namespace std;


IndexerStateMgr* IndexerStateMgr::m_instance = nullptr;
IndexerStateMgr* IndexerStateMgr::GetInstance()
{
	if ( IndexerStateMgr::m_instance == nullptr )
	{
	    auto mechFactory = MechanismFactory::GetMechanismFactory();
	    auto indexer = mechFactory->GetIndexer();
        if (indexer != nullptr)
        {
    		IndexerStateMgr::m_instance = new IndexerStateMgr();
        }
	}
	return IndexerStateMgr::m_instance;
}


/// @brief    initialize the state manager, parse the configuration file and create the states.
IndexerStateMgr::IndexerStateMgr() : StateMgr(),
                                     m_indexer(MechanismFactory::GetMechanismFactory()->GetIndexer()),
                                     m_shooter(MechanismFactory::GetMechanismFactory()->GetShooter()),
                                     m_shooterStateMgr(ShooterStateMgr::GetInstance())
{
    map<string, StateStruc> stateMap;
    stateMap[m_indexerOffXmlString] = m_offState;
    stateMap[m_indexerIndexLeftXmlString]  = m_indexLeftState;
    stateMap[m_indexerIndexRightXmlString] = m_indexRightState;
    stateMap[m_indexerExpelLeftXmlString]  = m_expelLeftState;
    stateMap[m_indexerExpelRightXmlString] = m_expelRightState;

    Init(MechanismFactory::GetMechanismFactory()->GetIndexer(), stateMap);
}   


/// @brief  run the current state
/// @return void
void IndexerStateMgr::CheckForStateTransition()
{
    if ( m_indexer != nullptr )
    {
        auto currentState = static_cast<INDEXER_STATE>(GetCurrentState());
        auto targetState = currentState;

        auto controller = TeleopControl::GetInstance();
        bool ballPresent = m_indexer->IsBallPresent();

        Logger::GetLogger()->ToNtTable(m_indexer->GetNetworkTableName(), string("Ball Present"), ballPresent ? string("true") : string("false"));

        if (m_shooterStateMgr != nullptr)
        {
            auto shooterState = static_cast<ShooterStateMgr::SHOOTER_STATE>(m_shooterStateMgr->GetCurrentState());

            if (m_shooter != nullptr)
            {
                auto isAtSpeed = m_shooterStateMgr->AtTarget();
                switch (shooterState)
                {
                    case ShooterStateMgr::SHOOTER_STATE::SHOOT_MANUAL:
                        [[fallthrough]]; //intentional fallthrough

                    case ShooterStateMgr::SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_CLOSE:
                        [[fallthrough]]; //intentional fallthrough

                    case ShooterStateMgr::SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_FAR:
                        [[fallthrough]]; //intentional fallthrough

                    case ShooterStateMgr::SHOOTER_STATE::SHOOT_LOW_GOAL:
                        if (ballPresent)
                        {
                            if (isAtSpeed)
                            {
                                if (currentState == INDEXER_STATE::INDEX_LEFT || currentState == INDEXER_STATE::INDEX_RIGHT)
                                {
                                    targetState = currentState; // stay in current state (we're shooting, so keep the current state to keep balls consistent)
                                }
                                else
                                {
                                    targetState = INDEXER_STATE::INDEX_LEFT; // help keep shots consistent
                                }
                            }
                            else
                            {
                                targetState = INDEXER_STATE::OFF;  // have ball and not shooting, so no indexing needed
                            }
                        }
                        else if (currentState == INDEXER_STATE::INDEX_LEFT) 
                        {
                            targetState = INDEXER_STATE::INDEX_RIGHT;  // no ball, so switch side the indexer is indexing
                        }
                        else if (currentState == INDEXER_STATE::INDEX_RIGHT) 
                        {
                            targetState = INDEXER_STATE::INDEX_LEFT;  // no ball, so switch side the indexer is indexing
                        }
                        else 
                        {
                            targetState = INDEXER_STATE::INDEX_LEFT;  // no ball and indexer isn't indexing, so start indexing
                        }
                        break;
                
                    case ShooterStateMgr::SHOOTER_STATE::PREPARE_TO_SHOOT:
                        if (!ballPresent)
                        {
                            // switch side indexing
                            if (currentState == INDEXER_STATE::INDEX_LEFT)
                            {
                                targetState = INDEXER_STATE::INDEX_RIGHT;
                            }
                            else
                            {
                                targetState = INDEXER_STATE::INDEX_LEFT;
                            }
                        }
                        else
                        {
                            targetState = INDEXER_STATE::OFF;
                        }
                        break;

                    default:
                        targetState = INDEXER_STATE::OFF;
                        break;
                }
            }
        }

        // not changing states and we don't have a ball present, if we have a controller, see if it changes the state
        if (targetState == currentState && !ballPresent && controller != nullptr)
        {
            auto leftIntakeStateMgr = LeftIntakeStateMgr::GetInstance();
            auto rightIntakeStateMgr = RightIntakeStateMgr::GetInstance();

            auto leftIntakeState = leftIntakeStateMgr != nullptr ? static_cast<IntakeStateMgr::INTAKE_STATE>(leftIntakeStateMgr->GetCurrentState()) : IntakeStateMgr::INTAKE_STATE::OFF;
            if (leftIntakeState == IntakeStateMgr::INTAKE_STATE::OFF && controller != nullptr)
            {
                if (controller->IsButtonPressed(TeleopControl::INTAKE_LEFT))
                {
                    leftIntakeState = IntakeStateMgr::INTAKE_STATE::INTAKE;
                }
            }
            auto rightIntakeState = rightIntakeStateMgr != nullptr ? static_cast<IntakeStateMgr::INTAKE_STATE>(rightIntakeStateMgr->GetCurrentState()) : IntakeStateMgr::INTAKE_STATE::OFF;
            if (rightIntakeState == IntakeStateMgr::INTAKE_STATE::OFF && controller != nullptr)
            {
                if (controller->IsButtonPressed(TeleopControl::INTAKE_RIGHT))
                {
                    rightIntakeState = IntakeStateMgr::INTAKE_STATE::INTAKE;
                }
            }
            
            // if we are intaking from a side, try indexing from that side too
            if (leftIntakeState == IntakeStateMgr::INTAKE_STATE::INTAKE)
            {
                targetState = INDEXER_STATE::INDEX_LEFT;
            }
            else if (rightIntakeState == IntakeStateMgr::INTAKE_STATE::INTAKE)
            {
                targetState = INDEXER_STATE::INDEX_RIGHT;
            }
        }
       
        if (targetState != currentState)
        {
            SetCurrentState(targetState, true);
        }
    }    
}
