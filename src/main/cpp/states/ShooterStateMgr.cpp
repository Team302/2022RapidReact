#include <map>
#include <memory>
#include <vector>

// FRC includes
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>

// Team 302 includes
#include <states/IState.h>
#include <states/ShooterStateMgr.h>
#include <xmlmechdata/StateDataDefn.h>
#include <controllers/MechanismTargetData.h>
#include <utils/Logger.h>
#include <gamepad/TeleopControl.h>
#include <states/ShooterState.h>
#include <states/StateMgr.h>
#include <states/StateStruc.h>
#include <subsys/MechanismFactory.h>
#include <subsys/MechanismTypes.h>


// Third Party Includes

using namespace std;


ShooterStateMgr* ShooterStateMgr::m_instance = nullptr;
ShooterStateMgr* ShooterStateMgr::GetInstance()
{
	if ( ShooterStateMgr::m_instance == nullptr )
	{
		ShooterStateMgr::m_instance = new ShooterStateMgr();
	}
	return ShooterStateMgr::m_instance;
}


/// @brief    initialize the state manager, parse the configuration file and create the states.
ShooterStateMgr::ShooterStateMgr()
{
    map<string, StateStruc> stateMap;
    stateMap["SHOOTEROFF"] = m_offState;
    stateMap["SHOOTERON"] = m_onState;
    

    Init(MechanismFactory::GetMechanismFactory()->GetShooter(), stateMap);
}   


/// @brief  run the current state
/// @return void
void ShooterStateMgr::CheckForStateTransition()
{
    if ( MechanismFactory::GetMechanismFactory()->GetShooter() != nullptr )
    {
        // process teleop/manual interrupts
        // auto currentState = static_cast<SHOOTER_STATE>(GetCurrentState());
    
        // auto controller = TeleopControl::GetInstance();
        // if ( controller != nullptr )
        // {
        //     auto intakePressed = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::INTAKE);
        //     auto expelPressed = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::EXPEL);
        //     if (intakePressed  &&  currentState != SHOOTER_STATE::ON )
        //     {
        //         SetCurrentState( INTAKE_STATE::INTAKE, false );
        //     }
        //     else if (expelPressed && currentState != INTAKE_STATE::EXPEL )
        //     {
        //         SetCurrentState( INTAKE_STATE::EXPEL, false );
        //     }           
        //     else if ((!intakePressed && !expelPressed) && currentState != INTAKE_STATE::OFF )
        //     {
        //         SetCurrentState( INTAKE_STATE::OFF, false );
        //     }
        // }
    }

}
