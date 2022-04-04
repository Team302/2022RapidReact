
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
#include <states/lift/LiftStateMgr.h>
#include <states/lift/LiftState.h>
#include <states/StateStruc.h>
#include <states/StateMgr.h>

#include <subsys/MechanismFactory.h>
#include <subsys/MechanismTypes.h>

#include <utils/Logger.h>


// Third Party Includes

using namespace std;


LiftStateMgr* LiftStateMgr::m_instance = nullptr;
LiftStateMgr* LiftStateMgr::GetInstance()
{
	if ( LiftStateMgr::m_instance == nullptr )
	{
	    auto mechFactory = MechanismFactory::GetMechanismFactory();
	    auto lift = mechFactory->GetLift();
        if (lift != nullptr)
        {
		    LiftStateMgr::m_instance = new LiftStateMgr();
        }
	}
	return LiftStateMgr::m_instance;
}

/// @brief    initialize the state manager, parse the configuration file and create the states.
LiftStateMgr::LiftStateMgr() : StateMgr(),
                                m_shooterStateMgr(ShooterStateMgr::GetInstance()),
                                m_shooter(MechanismFactory::GetMechanismFactory()->GetShooter()),
                                m_lift(MechanismFactory::GetMechanismFactory()->GetLift())
{
    map<string, StateStruc> stateMap;
    stateMap["LIFT_OFF"] = m_offState;
    stateMap["LIFT_LIFT"]  = m_liftState;
    stateMap["LIFT_LOWER"] = m_lowerState;
    Init(MechanismFactory::GetMechanismFactory()->GetLift(), stateMap);
}   


/// @brief  run the current state
/// @return void
void LiftStateMgr::CheckForStateTransition()
{
    if (m_lift != nullptr)
    {
        auto currentState = static_cast<LIFT_STATE>(GetCurrentState());
        auto targetState = currentState;

        if (m_shooterStateMgr != nullptr)
        {
            bool isAtSpeed = m_shooterStateMgr->AtTarget();

            if (isAtSpeed)
            {
                auto isShooting = m_shooterStateMgr->IsShooting();
                if (isShooting)
                {
                    targetState = LIFT_STATE::LIFT;
                }
                else
                {
                    targetState = LIFT_STATE::OFF;
                }
            }
            else
            {
                targetState = LIFT_STATE::OFF;
            }
        }
        else
        {
            targetState = LIFT_STATE::OFF;
            Logger::GetLogger()->LogError(string("LiftStateMgr"), string("No shooter state mgr"));
        }
    
        if (targetState != currentState)
        {
            SetCurrentState(targetState, true);
        }
    }
}
