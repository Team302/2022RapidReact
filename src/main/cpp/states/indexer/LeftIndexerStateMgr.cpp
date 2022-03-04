
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
#include <gamepad/TeleopControl.h>
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
LeftIndexerStateMgr::LeftIndexerStateMgr() : IndexerStates()
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
    if ( MechanismFactory::GetMechanismFactory()->GetLeftIndexer() != nullptr )
    {

    }    
}
