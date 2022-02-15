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
#include <states/intake/IntakeStateMgr.h>
#include <xmlmechdata/StateDataDefn.h>
#include <controllers/MechanismTargetData.h>
#include <utils/Logger.h>
#include <gamepad/TeleopControl.h>
#include <states/intake/IntakeState.h>
#include <states/StateMgr.h>
#include <states/StateStruc.h>
#include <subsys/MechanismFactory.h>
#include <subsys/MechanismTypes.h>
#include <states/servo/ServoStateMgr.h>


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
void IntakeStateMgr::CheckForStateTransition()
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
                SetCurrentState( SERVO_STATE::LOOK_RIGHT, false );
            }
            else if (leftPressed && currentState != INTAKE_STATE::LOOK_LEFT )
            {
                SetCurrentState( SERVO_STATE::LOOK_LEFT, false );
            }           
        }
    }

}




