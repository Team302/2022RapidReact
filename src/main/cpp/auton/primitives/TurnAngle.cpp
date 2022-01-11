
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
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

// FRC includes
#include <frc/Timer.h>
#include <frc/kinematics/ChassisSpeeds.h>

// Team 302 includes
#include <auton/primitives/TurnAngle.h>
#include <auton/PrimitiveParams.h>
#include <auton/primitives/IPrimitive.h>
#include <subsys/ChassisFactory.h>
#include <controllers/ControlModes.h>
#include <subsys/interfaces/IChassis.h>
#include <utils/Logger.h>
#include <hw/factories/PigeonFactory.h>
#include <hw/DragonPigeon.h>

// Third Party Includes


using namespace std;
using namespace frc;

//Team302 includes

TurnAngle::TurnAngle() : m_chassis( ChassisFactory::GetChassisFactory()->GetIChassis()),
						 m_timer( make_unique<Timer>() ),
						 m_targetAngle(0.0),
						 m_maxTime(0.0),
						 m_leftPos(0.0),
						 m_rightPos(0.0),
						 m_isDone(false),
						 m_pigeon(PigeonFactory::GetFactory()->GetPigeon()),
						 m_heading(0.0)
{
}

void TurnAngle::Init(PrimitiveParams* params) 
{
	m_isDone = false;
	auto startHeading = 0.0;
	auto pigeon = PigeonFactory::GetFactory()->GetPigeon();
	if ( pigeon != nullptr )
	{
		startHeading = pigeon->GetYaw();
		m_targetAngle = startHeading + params->GetHeading();
	}

	auto cd = make_shared<ControlData>( ControlModes::CONTROL_TYPE::POSITION_INCH, 
									ControlModes::CONTROL_RUN_LOCS::MOTOR_CONTROLLER,
									string("TurnAngle"),
									3.0,
									0.0,
									0.0,
									0.0,
									0.0,
									0.0,
									0.0,
									1.0,
									0.0   );

	m_maxTime = params->GetTime();
	m_timer->Reset();
	m_timer->Start();
}

void TurnAngle::Run() //best method ever. Does nothing, and should do nothing... NOT ANYMORE, BUDDY!
{
	if ( m_pigeon != nullptr )
	{
		m_heading = m_pigeon->GetYaw();
	}

	bool sign = (m_targetAngle - m_heading) > 0.0;

	ChassisSpeeds speeds;
	speeds.vx = 0_mps;
	speeds.vy = 0_mps;
	speeds.omega = sign ? units::degrees_per_second_t(90.0) : units::degrees_per_second_t(-90.0);
	m_chassis->Drive(speeds);
}

bool TurnAngle::IsDone() 
{
	if (!m_isDone) 
	{
		if ( m_pigeon != nullptr )
		{
			m_heading = m_pigeon->GetYaw();
		}
		if (abs(m_targetAngle - m_heading) < ANGLE_THRESH) 
		{
			m_isDone = true;
			ChassisSpeeds speeds;
			speeds.vx = 0_mps;
			speeds.vy = 0_mps;
			speeds.omega = units::degrees_per_second_t(0.0);
			m_chassis->Drive(speeds);
		}
	}
	return m_isDone; //|| m_timer->HasPeriodPassed( m_maxTime );
}
