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
#include <hw/DragonLimelight.h>
#include <hw/factories/LimelightFactory.h>
#include <states/IState.h>
#include <states/shooter/ShooterStateAutoHigh.h>
#include <subsys/MechanismFactory.h>
#include <subsys/Shooter.h>

// Third Party Includes


ShooterStateAutoHigh::ShooterStateAutoHigh
(
    ControlData*                    control, 
    ControlData*                    control2,
    double                          primaryTarget,
    double                          secondaryTarget

) : ShooterState(control, control2, primaryTarget, secondaryTarget), 
    m_dragonLimeLight(LimelightFactory::GetLimelightFactory()->GetLimelight()), 
    m_shooterTarget(primaryTarget)
{
}

void ShooterStateAutoHigh::Init() 
{
    if (GetShooter() != nullptr)
    {
        GetShooter()->SetControlConstants( 0, GetPrimaryControlData() );
        GetShooter()->SetSecondaryControlConstants( 0, GetSecondaryControlData() );
        double inches = 75.0;
        if (m_dragonLimeLight != nullptr)
        {
            auto distance = m_dragonLimeLight->EstimateTargetDistance();
            inches = distance.to<double>();
        }
        m_shooterTarget = 0.0021 * inches*inches - 0.3585 * inches + 72.867;
        GetShooter()->UpdateTargets(m_shooterTarget, GetSecondaryTarget());
    }
}
