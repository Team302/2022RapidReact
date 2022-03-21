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


// C++ Includes

// FRC includes

// Team 302 includes
#include <controllers/ControlData.h>
#include <states/shooter/ShooterState.h>
#include <states/Mech2MotorState.h>
#include <subsys/MechanismFactory.h>

// Third Party Includes


ShooterState::ShooterState
(
    ControlData*                    control, 
    ControlData*                    control2,
    double                          primaryTarget,
    double                          secondaryTarget
) : Mech2MotorState( MechanismFactory::GetMechanismFactory()->GetShooter(), control, control2, primaryTarget, secondaryTarget),
    m_shooter(MechanismFactory::GetMechanismFactory()->GetShooter())
{
}

bool ShooterState::AtTarget() const
{
    if (m_shooter != nullptr)
    {
        auto shootermotor = m_shooter->GetPrimaryMotor();
        auto rps = shootermotor.get()->GetRPS();
        auto target = m_shooter->GetPrimaryTarget();

        auto shootermotor2 = m_shooter->GetSecondaryMotor();
        auto rps2 = shootermotor2.get()->GetRPS();
        auto target2 = m_shooter->GetSecondaryTarget();

        return (abs(rps - target) < 1.5 && abs(rps2 - target2) < 1.5);
    }
    return true;
}