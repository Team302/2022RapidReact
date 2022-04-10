
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
                                     m_prevIndexState(INDEXER_STATE::OFF),
                                     m_loopsWithBallPresent(0),
                                     m_loopsToCenterBall(0),
                                     m_keepCurrentState(false)
{
    map<string, StateStruc> stateMap;
    stateMap[m_indexerOffXmlString] = m_offState;
    stateMap[m_indexerIndexLeftXmlString]  = m_indexLeftState;
    stateMap[m_indexerIndexRightXmlString] = m_indexRightState;
    stateMap[m_indexerExpelLeftXmlString]  = m_expelLeftState;
    stateMap[m_indexerExpelRightXmlString] = m_expelRightState;
    stateMap[m_indexerIndexBothXmlString] = m_indexBothState;

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

        auto controller = TeleopControl::GetInstance();

        Logger::GetLogger()->ToNtTable(m_indexer->GetNetworkTableName(), string("Ball Present"), ballPresent ? string("true") : string("false"));

        if (controller != nullptr && controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::MANUAL_INDEX))
        {
            targetState = INDEXER_STATE::INDEX_BOTH;
        }
        else if(!ballPresent)
        {
            if (IsIntakingLeft() && IsIntakingRight())
            {
                targetState = INDEXER_STATE::INDEX_BOTH;
            }
            else if(IsIntakingLeft())
            {
                targetState = INDEXER_STATE::INDEX_LEFT;
            }
            else if(IsIntakingRight())
            {
                targetState = INDEXER_STATE::INDEX_RIGHT;
            }
            else if(m_shooterStateMgr != nullptr && m_shooterStateMgr->IsShooting())
            {
                targetState = INDEXER_STATE::INDEX_BOTH;
            }
            else
            {
                targetState = INDEXER_STATE::OFF;// Not indexing nor shooting, so no indexing needed
            }
            m_loopsWithBallPresent = 0;  
        }
        else if(m_loopsWithBallPresent<NUM_LOOPS_TO_CENTER_BALL)
        {
            m_loopsWithBallPresent++;

            if (IsIntakingLeft() && IsIntakingRight())
            {
                targetState = INDEXER_STATE::INDEX_BOTH;
            }
            else if(IsIntakingLeft())
            {
                targetState = INDEXER_STATE::INDEX_LEFT;
            }
            else if(IsIntakingRight())
            {
                targetState = INDEXER_STATE::INDEX_RIGHT;
            }
        }
        else
        {
            targetState = INDEXER_STATE::OFF;  // have ball and not shooting, so no indexing needed
            
        }
        if(targetState != currentState)
        {
            SetCurrentState(targetState,true);
        }
    }    
	
}

bool IndexerStateMgr::IsBallPresent() const
{
    return m_indexer != nullptr ? m_indexer->IsBallPresent() : true;
}

bool IndexerStateMgr::IsIntakingLeft() const
{
    auto leftIntakeStateMgr = LeftIntakeStateMgr::GetInstance();
    return leftIntakeStateMgr != nullptr ? leftIntakeStateMgr->IsIntaking() : false;
}

bool IndexerStateMgr::IsIntakingRight() const
{
    auto rightIntakeStateMgr = RightIntakeStateMgr::GetInstance();
    return rightIntakeStateMgr != nullptr ? rightIntakeStateMgr->IsIntaking() : false;
}
