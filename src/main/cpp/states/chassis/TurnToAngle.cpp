
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

#include <cmath>
#include <states/chassis/TurnToAngle.h>
#include <states/IState.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/ProfiledPIDController.h>
#include <utils/AngleUtils.h>

#include <subsys/ChassisFactory.h>
#include <subsys/SwerveChassis.h>

using namespace frc;
using namespace std;

TurnToAngle::TurnToAngle
(
    units::angle::degree_t  targetAngle
) : m_targetAngle(targetAngle),
    m_chassis(ChassisFactory::GetChassisFactory()->GetSwerveChassis()),
    m_atTarget(false)
{
}

void TurnToAngle::Init()
{
}

// Worst case scenario
//
//
//                current orientation (0)
//                 +---------------+
//                 | @           @ | 
//                 |       ^       | 
//                 |       |       | 
//                 |               | 
//                 |               | 
//                 | @           @ | 
//                 +---------------+
//
//                desired orientation (180 / -180) worse case scenario
//                 +---------------+
//                 | @           @ | 
//                 |       |       | 
//                 |      \/       | 
//                 |               | 
//                 |               | 
//                 | @           @ | 
//                 +---------------+
//
//                desired orientation (+90)
//                 +---------------+
//                 | @           @ | 
//                 |               | 
//                 |     <--       | 
//                 |               | 
//                 |               | 
//                 | @           @ | 
//                 +---------------+
//
//                desired orientation (-90)
//                 +---------------+
//                 | @           @ | 
//                 |               | 
//                 |     -->       | 
//                 |               | 
//                 |               | 
//                 | @           @ | 
//                 +---------------+
//
//
void TurnToAngle::Run() 
{
    if (m_chassis != nullptr)
    {
        auto currentAngle = m_chassis->GetYaw();
        auto delta = AngleUtils::GetDeltaAngle(currentAngle, m_targetAngle);
        if (std::abs(delta.to<double>()) > m_angleTolerance.to<double>())
        {
            m_pid.SetSetpoint(m_targetAngle.to<double>());
            auto rotatePercent = m_pid.Calculate(currentAngle.to<double>());
            rotatePercent = clamp(rotatePercent, -1.0, 1.0);
            m_chassis->Drive(0.0, 0.0, rotatePercent,
                             IChassis::CHASSIS_DRIVE_MODE::ROBOT_ORIENTED,
						     IChassis::HEADING_OPTION::TOWARD_GOAL);

        }
        else
        {
            m_atTarget = true;
            m_chassis->Drive(0.0, 0.0, 0.0,
                             IChassis::CHASSIS_DRIVE_MODE::ROBOT_ORIENTED,
						     IChassis::HEADING_OPTION::TOWARD_GOAL);

        }
    }
}

bool TurnToAngle::AtTarget() const
{

    return m_atTarget;
}