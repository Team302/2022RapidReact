
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
#include <states/intake/LeftIntakeStateMgr.h>
#include <states/StateStruc.h>
#include <subsys/MechanismFactory.h>
#include <subsys/MechanismTypes.h>

// Third Party Includes

using namespace std;


LeftIntakeStateMgr* LeftIntakeStateMgr::m_instance = nullptr;
LeftIntakeStateMgr* LeftIntakeStateMgr::GetInstance()
{
	if ( LeftIntakeStateMgr::m_instance == nullptr )
	{
		LeftIntakeStateMgr::m_instance = new LeftIntakeStateMgr();
	}
	return LeftIntakeStateMgr::m_instance;
}


/// @brief    initialize the state manager, parse the configuration file and create the states.
LeftIntakeStateMgr::LeftIntakeStateMgr() : IntakeStateMgr()
{
    map<string, StateStruc> stateMap;
    stateMap["INTAKE_OFF"] = m_offState;
    stateMap["INTAKE_ON"]  = m_intakeState;
    stateMap["INTAKE_EXPEL"] = m_expelState;
    

    Init(MechanismFactory::GetMechanismFactory()->GetLeftIntake(), stateMap);
}   


/// @brief  run the current state
/// @return void
void LeftIntakeStateMgr::CheckForStateTransition()
{
    if ( MechanismFactory::GetMechanismFactory()->GetLeftIntake() != nullptr )
    {
        // process teleop/manual interrupts
        auto currentState = static_cast<INTAKE_STATE>(GetCurrentState());
        auto controller = TeleopControl::GetInstance();
        if ( controller != nullptr )
        {
            auto intakePressed = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::INTAKE_LEFT) || controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::INTAKE_LEFT_OLD);
            auto expelPressed = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::EXPEL_LEFT);
            if (intakePressed  &&  currentState != INTAKE_STATE::INTAKE )
            {
                SetCurrentState( INTAKE_STATE::INTAKE, true );
            }
            else if (expelPressed && currentState != INTAKE_STATE::EXPEL )
            {
                SetCurrentState( INTAKE_STATE::EXPEL, true );
            }           
            else if ((!intakePressed && !expelPressed) && currentState != INTAKE_STATE::OFF)
            {
                    SetCurrentState( INTAKE_STATE::OFF, true );
            }
            auto intake = MechanismFactory::GetMechanismFactory()->GetLeftIntake();
            auto stopped = intake->StopIfFullyExtended();
            if (!stopped)
            {
                intake->StopIfRetracted();
            }
        }
    }    
}
