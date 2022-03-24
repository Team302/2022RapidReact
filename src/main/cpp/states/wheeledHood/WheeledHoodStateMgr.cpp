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
#include <hw/factories/LimelightFactory.h>
#include <states/IState.h>
#include <states/shooter/ShooterStateMgr.h>
#include <states/StateMgr.h>
#include <states/StateStruc.h>
#include <states/wheeledHood/WheeledHoodStateMgr.h>
#include <subsys/MechanismFactory.h>
#include <subsys/MechanismTypes.h>
#include <subsys/Shooter.h>
#include <utils/Logger.h>
#include <xmlmechdata/StateDataDefn.h>


// Third Party Includes

using namespace std;


WheeledHoodStateMgr* WheeledHoodStateMgr::m_instance = nullptr;
WheeledHoodStateMgr* WheeledHoodStateMgr::GetInstance()
{
	if ( WheeledHoodStateMgr::m_instance == nullptr )
	{
	    auto mechFactory = MechanismFactory::GetMechanismFactory();
	    auto shooter = mechFactory->GetShooter();
	    if (shooter != nullptr)
        {
		    WheeledHoodStateMgr::m_instance = new WheeledHoodStateMgr();
        }
	}
	return WheeledHoodStateMgr::m_instance;
    
}


/// @brief    initialize the state manager, parse the configuration file and create the states.
WheeledHoodStateMgr::WheeledHoodStateMgr() : StateMgr(),
                                             m_nt()
{
    map<string, StateStruc> stateMap;
    stateMap[ShooterStateMgr::GetInstance()->m_shooterOffXmlString] = m_offState;
    stateMap[ShooterStateMgr::GetInstance()->m_shooterHighGoalCloseXmlString] = m_shootFarState;
    stateMap[ShooterStateMgr::GetInstance()->m_shooterHighGoalFarXmlString] = m_shootCloseState;
    stateMap[ShooterStateMgr::GetInstance()->m_shooterLowGoalXmlString] = m_shootLowState;
    stateMap[ShooterStateMgr::GetInstance()->m_shooterManualXmlString] = m_manualShootState;
    stateMap[ShooterStateMgr::GetInstance()->m_shooterPrepareXmlString] = m_prepareToShoot;   

    Init(MechanismFactory::GetMechanismFactory()->GetWheeledHood(), stateMap);
}   


void WheeledHoodStateMgr::CheckForStateTransition()
{
    if (MechanismFactory::GetMechanismFactory()->GetWheeledHood() != nullptr)
    {    
        auto currentState = GetCurrentState();
        auto targetState = ShooterStateMgr::GetInstance()->GetCurrentState();

        if (targetState != currentState)
        {
            SetCurrentState(targetState, true);
        }
        
    }
}
