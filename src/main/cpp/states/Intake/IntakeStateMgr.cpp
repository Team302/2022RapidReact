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


// Third Party Includes

using namespace std;


IntakeStateMgr* IntakeStateMgr::m_instance = nullptr;
IntakeStateMgr* IntakeStateMgr::GetInstance()
{
	if ( IntakeStateMgr::m_instance == nullptr )
	{
		IntakeStateMgr::m_instance = new IntakeStateMgr();
	}
	return IntakeStateMgr::m_instance;
}


/// @brief    initialize the state manager, parse the configuration file and create the states.
IntakeStateMgr::IntakeStateMgr()
{
    map<string, StateStruc> stateMap;
    //stateMap["INTAKEOFF"] = m_offState;
    stateMap["INTAKEON"] = m_intakeState;
   // stateMap["INTAKEEXPEL"] = m_expelState;

    Init(MechanismFactory::GetMechanismFactory()->GetIntake(), stateMap);
}   


/// @brief  run the current state
/// @return void
void IntakeStateMgr::CheckForStateTransition()
{
    if ( MechanismFactory::GetMechanismFactory()->GetIntake() != nullptr )
    {
        // process teleop/manual interrupts
        auto currentState = static_cast<INTAKE_STATE>(GetCurrentState());
    
        auto controller = TeleopControl::GetInstance();
        if ( controller != nullptr )
        {
            auto intakePressed = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::INTAKE);
            //auto expelPressed = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::EXPEL);
            if (intakePressed  &&  currentState != INTAKE_STATE::INTAKE )
            {
                SetCurrentState( INTAKE_STATE::INTAKE, false );
            }
            /*else if (expelPressed && currentState != INTAKE_STATE::EXPEL )
            {
                SetCurrentState( INTAKE_STATE::EXPEL, false );
            }           
            else if ((!intakePressed && !expelPressed) && currentState != INTAKE_STATE::OFF )
            {
                SetCurrentState( INTAKE_STATE::OFF, false );
            }
            */
        }
        
    }
    
}
