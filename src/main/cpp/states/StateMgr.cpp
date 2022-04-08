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
#include <memory>
#include <vector>

// FRC includes
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>

// Team 302 includes
#include <controllers/MechanismTargetData.h>
#include <states/climber/ClimberState.h>
#include <states/climber/ClimberManualState.h>
#include <states/indexer/IndexerState.h>
#include <states/intake/IntakeState.h>
#include <states/intake/ManualLeftIntakeState.h>
#include <states/intake/ManualRightIntakeState.h>
#include <states/IState.h>
#include <states/lift/LiftState.h>
#include <states/shooter/ShooterState.h>
#include <states/shooter/ShooterStateAutoHigh.h>
#include <states/shooter/ShooterStateManual.h>
#include <states/StateMgr.h>
#include <states/StateStruc.h>
#include <subsys/interfaces/IMech.h>
#include <subsys/MechanismFactory.h>
#include <utils/Logger.h>
#include <xmlmechdata/StateDataDefn.h>

// Third Party Includes

using namespace std;



/// @brief    initialize the state manager, parse the configuration file and create the states.
StateMgr::StateMgr() : m_mech(nullptr),
                       m_currentState(),
                       m_stateVector(),
                       m_currentStateID(0)
{
}
void StateMgr::Init
(
    IMech*                                  mech,
    const map<string,StateStruc>&           stateMap
) 
{
    m_mech = mech;


    if (mech != nullptr)
    {
        // Parse the configuration file 
        auto stateXML = make_unique<StateDataDefn>();
        vector<MechanismTargetData*> targetData = stateXML.get()->ParseXML(mech->GetType());

        if (targetData.empty())
        {
            Logger::GetLogger()->LogError(Logger::LOGGER_LEVEL::ERROR, mech->GetControlFileName(), string("No states"));
        }
        else
        {
            // initialize the xml string to state map
            m_stateVector.resize(stateMap.size());
            // create the states passing the configuration data
            for ( auto td: targetData )
            {
                auto stateString = td->GetStateString();
                auto stateStringToStrucItr = stateMap.find( stateString );
                if ( stateStringToStrucItr != stateMap.end() )
                {
                    auto struc = stateStringToStrucItr->second;
                    auto slot = struc.id;
                    if ( m_stateVector[slot] == nullptr )
                    {
                        auto controlData = td->GetController();
                	    auto controlData2 = td->GetController2();
                        auto target = td->GetTarget();
                	    auto secondaryTarget = td->GetSecondTarget();
                        auto robotPitch = td->GetRobotPitch();
                        auto function1Coeff = td->GetFunction1Coeff();
                        auto function2Coeff = td->GetFunction2Coeff();
                        auto type = struc.type;
                        IState* thisState = nullptr;
                        switch (type)
                        {
                            case StateType::LEFT_INTAKE:
                        	    thisState = new IntakeState(MechanismFactory::GetMechanismFactory()->GetLeftIntake(),
                                                            controlData,
                                                            controlData2, 
                                                            target, 
                                                            secondaryTarget);
                        	    break;

                            case StateType::LEFT_INTAKE_MANUAL:
                        	    thisState = new ManualLeftIntakeState(MechanismFactory::GetMechanismFactory()->GetLeftIntake(),
                                                                                                controlData,
                                                                                                controlData2, 
                                                                                                target, 
                                                                                                secondaryTarget);
                        	    break;

                            case StateType::RIGHT_INTAKE:
                        	    thisState = new IntakeState(MechanismFactory::GetMechanismFactory()->GetRightIntake(),
                                                            controlData,
                                                            controlData2, 
                                                            target, 
                                                            secondaryTarget);
                        	    break;

                            case StateType::RIGHT_INTAKE_MANUAL:
                        	    thisState = new ManualRightIntakeState(MechanismFactory::GetMechanismFactory()->GetRightIntake(),
                                                                                                controlData,
                                                                                                controlData2, 
                                                                                                target, 
                                                                                                secondaryTarget);
                        	    break;
                        	    
                    	    case StateType::SHOOTER:
                       		    thisState = new ShooterState(controlData, 
                                                             controlData2, 
                                                             target, 
                                                             secondaryTarget);
                       		    break;

                    	    case StateType::SHOOTER_MANUAL:
                       		    thisState = new ShooterStateManual();
                       		    break;

                    	    case StateType::SHOOTER_AUTO:
                       		    thisState = new ShooterStateAutoHigh(controlData, 
                                                                     controlData2, 
                                                                     target, 
                                                                     secondaryTarget, 
                                                                     function1Coeff, 
                                                                     function2Coeff);
                       		    break;

                            case StateType::CLIMBER:
                                thisState = new ClimberState(controlData, 
                                                             controlData2, 
                                                             target, 
                                                             secondaryTarget,
                                                             robotPitch);
                                break;

                            case StateType::CLIMBER_MANUAL:
                                thisState = new ClimberManualState(controlData, 
                                                                   controlData2, 
                                                                   target, 
                                                                   secondaryTarget);
                                break;


                            case StateType::INDEXER:
                                thisState = new IndexerState(MechanismFactory::GetMechanismFactory()->GetIndexer(), 
                                                             controlData, 
                                                             controlData2,
                                                             target,
                                                             secondaryTarget);
                                break;
                                
                            case StateType::LIFT:
                                thisState = new LiftState(MechanismFactory::GetMechanismFactory()->GetLift(), controlData, target);
                                break;

                    	    default:
                        	    Logger::GetLogger()->LogError( string("StateMgr::StateMgr"), string("unknown state"));
                    	        break;
                	    }
                	    if (thisState != nullptr)
                	    {
                    	    m_stateVector[slot] = thisState;
                            if (struc.isDefault)
                            {
                        	    m_currentState = thisState;
                        	    m_currentStateID = slot;
                        	    m_currentState->Init();
                    	    }
                	    }
            	    }
            	    else
            	    {
                	    Logger::GetLogger()->LogError( string("StateMgr::StateMgr"), string("multiple mechanism state info for state"));
            	    }
        	    }
        	    else
        	    {
            	    Logger::GetLogger()->LogError( string("StateMgr::StateMgr"), string("state not found"));
                }
            }
        }
    }
}

/// @brief  run the current state
/// @return void
void StateMgr::RunCurrentState()
{
    if ( m_mech != nullptr )
    {
        CheckForStateTransition();

        // run the current state
        if ( m_currentState != nullptr )
        {
            m_currentState->Run();
        }
    }

}

void StateMgr::CheckForStateTransition()
{
    // override this method if joystick inputs could change states;  Format 
    // would look something like:
    //    auto controller = TeleopControl::GetInstance();
    //    if ( controller != nullptr )
    //    {
    //          code here that checks the inputs
    //    }
}

/// @brief  set the current state, initialize it and run it
/// @return void
void StateMgr::SetCurrentState
(
    int             stateID,
    bool            run
)
{
    if (m_mech != nullptr )
    {
        auto state = m_stateVector[stateID];
        if ( state != nullptr && state != m_currentState)
        {    
            m_currentState = state;
            m_currentStateID = stateID;       
            m_currentState->Init();
            
            if ( run )
            {
                m_currentState->Run();
            }
            
        }
    }
}




