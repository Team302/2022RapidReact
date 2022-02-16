//====================================================================================================================================================
/// Copyright 2022 Lake Orion Robotics FIRST Team 302 
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

// C++ Includes
#include <map>
#include <memory>
#include <vector>

// FRC includes
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>

// Team 302 includes
#include <states/IState.h>
#include <xmlmechdata/StateDataDefn.h>
#include <controllers/MechanismTargetData.h>
#include <utils/Logger.h>
#include <gamepad/TeleopControl.h>
#include <states/StateMgr.h>
#include <states/StateStruc.h>
#include <subsys/MechanismFactory.h>
#include <subsys/MechanismTypes.h>
#include <states/servo/ServoStateMgr.h>
#include <subsys/MechanismFactory.h>
#include <hw/DragonServo.h>


// Third Party Includes

using namespace std;


ServoStateMgr* ServoStateMgr::m_instance = nullptr;
ServoStateMgr* ServoStateMgr::GetInstance()
{
	if ( ServoStateMgr::m_instance == nullptr )
	{
	     ServoStateMgr::m_instance = new ServoStateMgr();
	}
	return ServoStateMgr::m_instance;
}


/// @brief    initialize the state manager, parse the configuration file and create the states.
ServoStateMgr::ServoStateMgr()
{
    map<string, StateStruc> stateMap;
    stateMap["LOOKRIGHT"] = m_rightState;
    stateMap["LOOKLEFT"] = m_leftState;

    Init(MechanismFactory::GetMechanismFactory()->GetServo(), stateMap);
}   


/// @brief  run the current state
/// @return void
void ServoStateMgr::CheckForStateTransition()
{
    if ( MechanismFactory::GetMechanismFactory()->GetServo() != nullptr )
    {
        // process teleop/manual interrupts
        auto currentState = static_cast<SERVO_STATE>(GetCurrentState());
    
        auto controller = TeleopControl::GetInstance();
        if ( controller != nullptr )
        {
            auto rightPressed = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::LOOK_RIGHT);
            auto leftPressed = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::LOOK_LEFT);
            if (rightPressed  &&  currentState != SERVO_STATE::LOOK_RIGHT )
            {
                SetCurrentState( SERVO_STATE::LOOK_RIGHT, true );
            }
            else if (leftPressed && currentState != SERVO_STATE::LOOK_LEFT )
            {
                SetCurrentState( SERVO_STATE::LOOK_LEFT, true );
            }           
        }
    }

}




