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
#include <subsys/Shooter.h>
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
ShooterStateMgr::ShooterStateMgr() : StateMgr(),
                                     m_shooter(MechanismFactory::GetMechanismFactory()->GetShooter()),
                                     m_nt()
{
    map<string, StateStruc> stateMap;
    stateMap[m_shooterOffXmlString] = m_offState;
    stateMap[m_shooterHighGoalCloseXmlString] = m_shootFarState;
    stateMap[m_shooterHighGoalFarXmlString] = m_shootCloseState;
    stateMap[m_shooterLowGoalXmlString] = m_shootLowState;
    stateMap[m_shooterManualXmlString] = m_manualShootState;
    stateMap[m_shooterHoodXmlString] = m_shooterHoodAdjust;
    stateMap[m_shooterPrepareXmlString] = m_prepareToShoot;

    m_dragonLimeLight = LimelightFactory::GetLimelightFactory()->GetLimelight();
    

    Init(m_shooter, stateMap);
    if (m_shooter != nullptr)
    {
        auto ntName = m_shooter->GetNetworkTableName();
        m_nt = nt::NetworkTableInstance::GetDefault().GetTable(ntName);
    }
    else
    {
        m_nt = nt::NetworkTableInstance::GetDefault().GetTable("shooter");
    }
}   


bool ShooterStateMgr::AtTarget() const
{
    Logger::GetLogger()->ToNtTable(m_nt, string("At Target"), GetCurrentStatePtr()->AtTarget() ? "true" : "false");
    return GetCurrentStatePtr()->AtTarget();
}

void ShooterStateMgr::CheckForStateTransition()
{

    if ( m_shooter != nullptr )
    {    
        auto currentState = static_cast<SHOOTER_STATE>(GetCurrentState());
        auto targetState = currentState;

        auto controller = TeleopControl::GetInstance();
        if ( controller != nullptr )
        {
            auto isShootHighSelected = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::AUTO_SHOOT_HIGH);
            auto isShootLowSelected  = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::AUTO_SHOOT_LOW);
            auto isManualShootSelected = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::MANUAL_SHOOT);
            auto isShooterOffSelected = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::SHOOTER_OFF);
            auto isPrepareToShootSelected = controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::SHOOTER_MTR_ON);
            auto shooterHoodAdjust = controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::SHOOTER_HOOD_MAN);
            if (isShootHighSelected && m_dragonLimeLight != nullptr)
            {
                Logger::GetLogger()->ToNtTable(m_nt, string("horizontal angle "), m_dragonLimeLight->GetTargetHorizontalOffset().to<double>());
                if(m_dragonLimeLight->GetTargetHorizontalOffset() <= 3.0_deg)
                {
                    Logger::GetLogger()->ToNtTable(m_nt, string("distance "), m_dragonLimeLight->EstimateTargetDistance().to<double>());
                    //if(m_dragonLimeLight->EstimateTargetDistance() >= units::length::inch_t(m_CHANGE_STATE_TARGET) && currentState != SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_FAR)
                    //{
                        targetState = SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_FAR;
                    //}
                    //else if (currentState != SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_CLOSE)
                    //{
                    //    targetState = SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_CLOSE;
                    //}
                }
            }
            else if (isShootLowSelected)
            {
                targetState = SHOOTER_STATE::SHOOT_LOW_GOAL;
            }
            else if (isPrepareToShootSelected)
            {
                targetState = SHOOTER_STATE::PREPARE_TO_SHOOT;
            }
            else if (isManualShootSelected)
            {
                targetState = SHOOTER_STATE::SHOOT_MANUAL;
            }
            else if (abs(shooterHoodAdjust) > 0.05) 
            {
                targetState = SHOOTER_STATE::SHOOTER_HOOD_ADJUST;
            }
            else if (isShooterOffSelected)
            {
                targetState = SHOOTER_STATE::OFF;
            }
            else if (currentState != SHOOTER_STATE::OFF)
            {
                targetState = SHOOTER_STATE::PREPARE_TO_SHOOT;
            }

        }

        if (targetState != currentState)
        {
            Logger::GetLogger()->ToNtTable(m_nt, string("Changing Shooter State"), targetState);
            SetCurrentState(targetState, true);
        }
    }
}
