
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
#include <memory>
#include <string>

// FRC includes
#include <frc/DriverStation.h>
#include <frc/Timer.h>

// Team 302 includes
#include <auton/AutonSelector.h>
#include <auton/CyclePrimitives.h>
#include <auton/PrimitiveEnums.h>
#include <auton/PrimitiveFactory.h>
#include <auton/PrimitiveParams.h>
#include <auton/PrimitiveParser.h>
#include <auton/primitives/IPrimitive.h>
#include <states/balltransfer/BallTransferStateMgr.h>
#include <states/intake/LeftIntakeStateMgr.h>
#include <states/intake/RightIntakeStateMgr.h>
#include <states/shooter/ShooterStateMgr.h>
#include <subsys/MechanismFactory.h>
#include <subsys/Intake.h>
#include <subsys/BallTransfer.h>
#include <subsys/Shooter.h>
#include <utils/Logger.h>

// Third Party Includes

using namespace frc;
using namespace std;

CyclePrimitives::CyclePrimitives() : m_primParams(), 
									 m_currentPrimSlot(0), 
								     m_currentPrim(nullptr), 
									 m_primFactory(
									 PrimitiveFactory::GetInstance()), 
									 m_doNothing(nullptr), 
									 m_autonSelector( new AutonSelector()) ,
									 m_timer( make_unique<Timer>()),
									 m_maxTime( 0.0 ),
									 m_isDone( false )
{
}

void CyclePrimitives::Init()
{
	m_currentPrimSlot = 0; //Reset current prim
	m_primParams.clear();

	m_primParams = PrimitiveParser::ParseXML( m_autonSelector->GetSelectedAutoFile() );
	if (!m_primParams.empty())
	{
		GetNextPrim();
	}
}

void CyclePrimitives::Run()
{
	Logger::GetLogger()->ToNtTable(string("auton"), string("run primitive"), string("cycle primitives"));
	if (m_currentPrim != nullptr)
	{
		m_currentPrim->Run();
		Logger::GetLogger()->ToNtTable(string("auton"), string("run primitive"), string("start"));
		
		auto mechFactory = MechanismFactory::GetMechanismFactory();
		auto leftIntake = mechFactory->GetLeftIntake();
		auto leftIntakeStateMgr = leftIntake != nullptr ? LeftIntakeStateMgr::GetInstance() : nullptr;

		auto rightIntake = mechFactory->GetRightIntake();
		auto rightIntakeStateMgr = rightIntake != nullptr ? RightIntakeStateMgr::GetInstance() : nullptr;

		auto ballTransfer = mechFactory->GetBallTransfer();
		auto ballTransferStateMgr = ballTransfer != nullptr ? BallTransferStateMgr::GetInstance() : nullptr;

		auto shooter = mechFactory->GetShooter();
		auto shooterStateMgr = shooter != nullptr ? ShooterStateMgr::GetInstance() : nullptr;
		
		if (leftIntakeStateMgr != nullptr)
		{
			Logger::GetLogger()->ToNtTable(string("auton"), string("run primitive"), string("left intake"));
			leftIntakeStateMgr->RunCurrentState();
		}
		if (rightIntakeStateMgr != nullptr)
		{
			Logger::GetLogger()->ToNtTable(string("auton"), string("run primitive"), string("right intake"));
			rightIntakeStateMgr->RunCurrentState();
		}
		if (shooterStateMgr != nullptr)
		{
			Logger::GetLogger()->ToNtTable(string("auton"), string("run primitive"), string("shooter"));
			shooterStateMgr->RunCurrentState();
		}
		if (ballTransferStateMgr != nullptr)
		{
			Logger::GetLogger()->ToNtTable(string("auton"), string("run primitive"), string("ball transfer"));
			ballTransferStateMgr->RunCurrentState();
		}

		if (m_currentPrim->IsDone())
		{
			GetNextPrim();
		}
	}
	else
	{
		m_isDone = true;
		m_primParams.clear();	// clear the primitive params vector
		m_currentPrimSlot = 0;  //Reset current prim slot
		RunDoNothing();
	}
}

bool CyclePrimitives::AtTarget() const
{
	return m_isDone;
}

void CyclePrimitives::GetNextPrim()
{
	PrimitiveParams* currentPrimParam = (m_currentPrimSlot < (int) m_primParams.size()) ? m_primParams[m_currentPrimSlot] : nullptr;

	m_currentPrim = (currentPrimParam != nullptr) ? m_primFactory->GetIPrimitive(currentPrimParam) : nullptr;
	if (m_currentPrim != nullptr)
	{
		m_currentPrim->Init(currentPrimParam);
		auto mechFactory = MechanismFactory::GetMechanismFactory();
		auto leftIntake = mechFactory->GetLeftIntake();
		auto leftIntakeStateMgr = leftIntake != nullptr ? LeftIntakeStateMgr::GetInstance() : nullptr;

		auto rightIntake = mechFactory->GetRightIntake();
		auto rightIntakeStateMgr = rightIntake != nullptr ? RightIntakeStateMgr::GetInstance() : nullptr;

		auto ballTransfer = mechFactory->GetBallTransfer();
		auto ballTransferStateMgr = ballTransfer != nullptr ? BallTransferStateMgr::GetInstance() : nullptr;

		auto shooter = mechFactory->GetShooter();
		auto shooterStateMgr = shooter != nullptr ? ShooterStateMgr::GetInstance() : nullptr;

		if (leftIntakeStateMgr != nullptr)
		{
			leftIntakeStateMgr->SetCurrentState(currentPrimParam->GetLeftIntakeState(), true);
		}
		if (rightIntakeStateMgr != nullptr)
		{
			rightIntakeStateMgr->SetCurrentState(currentPrimParam->GetRightIntakeState(), true);
		}
		if (shooterStateMgr != nullptr)
		{
			shooterStateMgr->SetCurrentState(currentPrimParam->GetShooterState(), true);
		}
		if (ballTransferStateMgr != nullptr)
		{
			ballTransferStateMgr->RunCurrentState();
		}
		m_maxTime = currentPrimParam->GetTime();
		m_timer->Reset();
		m_timer->Start();
	}

	m_currentPrimSlot++;
}

void CyclePrimitives::RunDoNothing()
{
	if (m_doNothing == nullptr)
	{	
		auto time = DriverStation::GetMatchType() != DriverStation::MatchType::kNone ? 
							 DriverStation::GetMatchTime() : 15.0;
		auto params = new PrimitiveParams( DO_NOTHING,          // identifier
		                                   time,              	// time
		                                   0.0,                 // distance
		                                   0.0,                 // target x location
		                                   0.0,                 // target y location
										   IChassis::HEADING_OPTION::MAINTAIN,
		                                   0.0,                 // heading
		                                   0.0,                 // start drive speed
		                                   0.0,					// end drive speed
										  std::string(),
										  IntakeStateMgr::INTAKE_STATE::OFF,
										  IntakeStateMgr::INTAKE_STATE::OFF,
										  ShooterStateMgr::SHOOTER_STATE::PREPARE_TO_SHOOT );             
		m_doNothing = m_primFactory->GetIPrimitive(params);
		m_doNothing->Init(params);
	}
	m_doNothing->Run();
}
