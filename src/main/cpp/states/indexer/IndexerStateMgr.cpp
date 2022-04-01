
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
                                     m_shooterStateMgr(ShooterStateMgr::GetInstance()),
                                     m_prevIndexStates(),
                                     m_loopsWithBallPresent(0),
                                     m_loopsToCenterBall(0),
                                     m_keepCurrentState(false)
{
    m_prevIndexStates[0] = INDEXER_STATE::OFF;
    m_prevIndexStates[1] = INDEXER_STATE::OFF;
    
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

        bool ballPresent = m_indexer->IsBallPresent();
        Logger::GetLogger()->ToNtTable(m_indexer->GetNetworkTableName(), string("Ball Present"), ballPresent ? string("true") : string("false"));

        if (IsIntakingLeft() && !ballPresent)
        {
            targetState = INDEXER_STATE::INDEX_LEFT;
            m_keepCurrentState = true;
        }
        else if (IsIntakingRight() && !ballPresent)
        {
            targetState = INDEXER_STATE::INDEX_RIGHT;
            m_keepCurrentState = true;
        }
        else if (!ballPresent)
        {
            m_loopsToCenterBall = 0;
        }
        else if (currentState == INDEXER_STATE::INDEX_LEFT ||  currentState == INDEXER_STATE::INDEX_RIGHT)
        {
            // need to keep indexing to center the ball
            m_keepCurrentState = m_loopsToCenterBall <= NUM_LOOPS_TO_CENTER_BALL;
            m_loopsToCenterBall++;
        }

        if (!m_keepCurrentState && m_shooterStateMgr != nullptr)
        {
            auto shooterState = static_cast<ShooterStateMgr::SHOOTER_STATE>(m_shooterStateMgr->GetCurrentState());
            if (IsShooting())
            {
                if (currentState == INDEXER_STATE::INDEX_LEFT || 
                    currentState == INDEXER_STATE::INDEX_RIGHT)
                {
                    targetState = currentState; // stay in current state (we're shooting, so keep the current state to keep balls consistent)
                }
                else
                {   // Set the state to the first state stored
                    targetState = m_prevIndexStates[1];
                    m_prevIndexStates[1] = m_prevIndexStates[0];
                }
                m_loopsWithBallPresent++;
                if (m_loopsWithBallPresent > NUM_LOOPS_TO_SHOOT_BALL)
                {
                    m_loopsWithBallPresent = 0;
                    if (currentState == INDEXER_STATE::INDEX_LEFT)
                    {
                        targetState = INDEXER_STATE::INDEX_RIGHT;
                    }
                    else if (currentState == INDEXER_STATE::INDEX_RIGHT)
                    {
                        targetState = INDEXER_STATE::INDEX_LEFT;
                    }
                }            
            }
            else if (ballPresent || shooterState == ShooterStateMgr::SHOOTER_STATE::PREPARE_TO_SHOOT
                                 || shooterState == ShooterStateMgr::SHOOTER_STATE::OFF)
            {
                targetState = INDEXER_STATE::OFF;  // have ball and not shooting, so no indexing needed
            }
            else if (currentState == INDEXER_STATE::INDEX_LEFT && !ballPresent) 
            {
                targetState = INDEXER_STATE::INDEX_RIGHT;  // no ball, so switch side the indexer is indexing
            }
            else if (currentState == INDEXER_STATE::INDEX_RIGHT && !ballPresent) 
            {
                targetState = INDEXER_STATE::INDEX_LEFT;  // no ball, so switch side the indexer is indexing
            }
            else if (!ballPresent)
            {
                targetState = INDEXER_STATE::INDEX_LEFT;  // no ball and indexer isn't indexing, so start indexing
            }
        }
      
        if (targetState != currentState)
        {
            SetCurrentState(targetState, true);
            if (targetState == INDEXER_STATE::INDEX_LEFT || targetState == INDEXER_STATE::INDEX_RIGHT)
            {
                m_prevIndexStates[1] = m_prevIndexStates[0];
                m_prevIndexStates[0] = targetState;
            }
        }
    }    
}

bool IndexerStateMgr::IsBallPresent() const
{
    return m_indexer != nullptr ? m_indexer->IsBallPresent() : true;
}

bool IndexerStateMgr::IsShooting() const
{
    if (m_shooterStateMgr != nullptr)
    {
        return (m_shooterStateMgr->IsShooting() && m_shooterStateMgr->AtTarget() && IsBallPresent());
    }
    return false;
}

bool IndexerStateMgr::IsIntakingLeft() const
{
    auto controller = TeleopControl::GetInstance();
    auto leftIntakeStateMgr = LeftIntakeStateMgr::GetInstance();

    auto leftIntakeState = leftIntakeStateMgr != nullptr ? static_cast<IntakeStateMgr::INTAKE_STATE>(leftIntakeStateMgr->GetCurrentState()) : IntakeStateMgr::INTAKE_STATE::OFF;
    if (leftIntakeState == IntakeStateMgr::INTAKE_STATE::OFF && controller != nullptr)
    {
        return controller->IsButtonPressed(TeleopControl::INTAKE_LEFT);
    }
    return false;
}

bool IndexerStateMgr::IsIntakingRight() const
{
    auto controller = TeleopControl::GetInstance();
    auto rightIntakeStateMgr = RightIntakeStateMgr::GetInstance();
    auto rightIntakeState = rightIntakeStateMgr != nullptr ? static_cast<IntakeStateMgr::INTAKE_STATE>(rightIntakeStateMgr->GetCurrentState()) : IntakeStateMgr::INTAKE_STATE::OFF;
    if (rightIntakeState == IntakeStateMgr::INTAKE_STATE::OFF && controller != nullptr)
    {
        return controller->IsButtonPressed(TeleopControl::INTAKE_RIGHT);
    }
    return false;
}
