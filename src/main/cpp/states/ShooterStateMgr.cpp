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
#include <map>
#include <memory>
#include <vector>

// FRC includes
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>

// Team 302 includes
#include <controllers/MechanismTargetData.h>
#include <gamepad/TeleopControl.h>
#include <hw/DragonLimelight.h>
#include <hw/factories/LimelightFactory.h>
#include <states/IState.h>
#include <states/shooter/ShooterState.h>
#include <states/shooter/ShooterStateMgr.h>
#include <states/StateMgr.h>
#include <states/StateStruc.h>
#include <subsys/MechanismFactory.h>
#include <subsys/MechanismTypes.h>
#include <utils/Logger.h>
#include <xmlmechdata/StateDataDefn.h>


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
    stateMap["SHOOTERCLOSE"] = m_shootFarState;
    stateMap["SHOOTERFAR"] = m_shootCloseState;
    m_dragonLimeLight = LimelightFactory::GetLimelightFactory()->GetLimelight();
    

    Init(MechanismFactory::GetMechanismFactory()->GetShooter(), stateMap);
}   


/// @brief  run the current state
/// @return void

bool ShooterStateMgr::AtTarget() const
{
    return GetCurrentStatePtr()->AtTarget();
}

void ShooterStateMgr::CheckForStateTransition()
{
    if(m_dragonLimeLight->GetTargetHorizontalOffset() <= 10.0_deg)
    {
        if(m_dragonLimeLight->EstimateTargetDistance() >= units::length::inch_t(m_CHANGE_STATE_TARGET))
        {
            SetCurrentState(ShooterStateMgr::SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_FAR, true);
        }
        else
        {
            SetCurrentState(ShooterStateMgr::SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_CLOSE, true);
        }
    }

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
