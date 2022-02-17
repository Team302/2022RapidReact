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
#include <states/IState.h>
#include <states/shooter/ShooterStateMgr.h>
#include <xmlmechdata/StateDataDefn.h>
#include <controllers/MechanismTargetData.h>
#include <utils/Logger.h>
#include <gamepad/TeleopControl.h>
#include <states/shooter/ShooterState.h>
#include <states/StateMgr.h>
#include <states/StateStruc.h>
#include <subsys/MechanismFactory.h>
#include <subsys/MechanismTypes.h>
#include <hw/DragonLimelight.h>
#include <hw/factories/LimelightFactory.h>


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
    stateMap["SHOOTLOWGOAL"] = m_shootLowState;
    stateMap["MANUALSHOOT"] = m_manualShootState;
    stateMap["ADJUSTHOOD"] = m_shooterHoodAdjust;
    stateMap["PREPARETOSHOOT"] = m_prepareToShoot;

    m_dragonLimeLight = LimelightFactory::GetLimelightFactory()->GetLimelight();
    

    Init(MechanismFactory::GetMechanismFactory()->GetShooter(), stateMap);
}   


/// @brief  run the current state
/// @return void

bool ShooterStateMgr::AtTarget() {return GetCurrentStatePtr()->AtTarget();}

void ShooterStateMgr::CheckForStateTransition()
{

    if ( MechanismFactory::GetMechanismFactory()->GetShooter() != nullptr )
    {    
        auto controller = TeleopControl::GetInstance();
        if ( controller != nullptr )
        {
            auto currentState = static_cast<SHOOTER_STATE>(GetCurrentState());

            auto isShootHighSelected = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::AUTO_SHOOT_HIGH);
            auto isShootLowSelected  = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::AUTO_SHOOT_LOW);
            auto isManualShootSelected = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::MANUAL_SHOOT);
            auto isShooterOffSelected = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::SHOOTER_OFF);
            auto isPrepareToShootSelected = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::SHOOTER_MTR_ON);
            auto shooterHoodAdjust = controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::SHOOTER_HOOD_MAN);
            if (isShootHighSelected && m_dragonLimeLight != nullptr)
            {
                if(m_dragonLimeLight->GetTargetHorizontalOffset() <= 10.0_deg)
                {
                    if(m_dragonLimeLight->EstimateTargetDistance() >= units::length::inch_t(m_CHANGE_STATE_TARGET) && currentState != SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_FAR)
                    {
                        SetCurrentState(SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_FAR, true);
                    }
                    else if (currentState != SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_CLOSE)
                    {
                        SetCurrentState(SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_CLOSE, true);
                    }
                }
            }
            else if (isShootLowSelected)
            {
                // TODO:: add state definition
            }
            else if (isShooterOffSelected && currentState != SHOOTER_STATE::OFF)
            {
                SetCurrentState(SHOOTER_STATE::OFF, true);
            }
            else if (isManualShootSelected)
            {
                // TODO: Define State
            }
            else if (abs(shooterHoodAdjust) > 0.05) // TODO: should this be an axis instead of a button?
            {
                // TODO: Define State
            }
            else if (isPrepareToShootSelected)
            {
                // TODO: Define State
            }
        }
    }
}
