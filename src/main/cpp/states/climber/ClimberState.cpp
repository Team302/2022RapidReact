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

#include <algorithm>
#include <cmath>

#include <units/length.h>
#include <units/angle.h>

#include <controllers/ControlData.h>
#include <controllers/MechanismTargetData.h>
#include <hw/factories/PigeonFactory.h>
#include <states/Climber/ClimberState.h>
#include <states/Mech2MotorState.h>
#include <subsys/MechanismFactory.h>

//Debugging
#include <utils/Logger.h>

using namespace std;

ClimberState::ClimberState
(
    ControlData*                    controlData, 
    ControlData*                    controlData2, 
    double                          target1,
    double                          target2,
    double                          robotPitch
) : Mech2MotorState( MechanismFactory::GetMechanismFactory()->GetClimber(), 
                     controlData, 
                     controlData2, 
                     target1, 
                     target2 ),
    m_climber(MechanismFactory::GetMechanismFactory()->GetClimber()),
    m_liftControlData(controlData),
    m_rotateControlData(controlData2),
    m_liftTarget(target1),
    m_rotateTarget(target2),
    m_robotPitch(robotPitch),
    m_liftController(new DragonPID(controlData)),
    m_rotateController(new DragonPID(controlData2)),
    m_liftMotor(m_climber->GetPrimaryMotor()),
    m_rotateMotor(m_climber->GetSecondaryMotor())
{
}

void ClimberState::Init()
{
    if (m_climber != nullptr)
    {
        m_climber->SetControlConstants(0, GetPrimaryControlData());
        m_climber->SetSecondaryControlConstants(0, GetSecondaryControlData());
        m_climber->UpdateTargets(m_liftTarget, m_rotateTarget);
    }
}

void ClimberState::Run()
{
    if (m_climber != nullptr)
    {
        Logger::GetLogger()->ToNtTable("climberNT", "Lift Height", GetLiftHeight());
        Logger::GetLogger()->ToNtTable("climberNT", "Rotate Angle", GetRotateAngle());
    }
}

bool ClimberState::AtTarget() const
{
    auto pigeon = PigeonFactory::GetFactory()->GetCenterPigeon();
    auto deltaPitch = pigeon != nullptr ? pigeon->GetPitch() - m_robotPitch : 0.0;
    return LiftTargetReached() && //Is lift within 1/4 of an inch of target?
           RotateTargetReached();
           //Debugging
           //&& //Is rotating arm within 2 degrees of target?
           //abs(deltaPitch) < 2.0; //Is robot pitch within 2 degrees of target?
}

bool ClimberState::LiftTargetReached() const
{
    if (m_liftTarget >= m_climber->GetMaxReach())
    {
        return m_climber->IsAtMaxReach(m_climber->GetPrimaryMotor(), m_climber->GetPrimaryPosition());
    }
    else if (m_liftTarget <= m_climber->GetMinReach())
    {
        return m_climber->IsAtMinReach(m_climber->GetPrimaryMotor(), m_climber->GetPrimaryPosition());
    }
    else
    {
        return abs(GetLiftHeight()-m_liftTarget) < 0.25;
    }
}

bool ClimberState::RotateTargetReached() const
{
    if (m_rotateTarget >= m_climber->GetMaxReach())
    {
        return m_climber->IsAtMaxRotation(m_climber->GetSecondaryMotor(), m_climber->GetSecondaryPosition());
    }
    else if (m_rotateTarget <= m_climber->GetMinReach())
    {
        return m_climber->IsAtMinRotation(m_climber->GetSecondaryMotor(), m_climber->GetSecondaryPosition());
    }
    else
    {
        return abs(GetRotateAngle()-m_rotateTarget) < 1.0;
    }
}

double ClimberState::GetLiftHeight() const
{
    if (m_climber != nullptr)
    {
        auto liftMotor = m_climber->GetPrimaryMotor();
        if (liftMotor.get() != nullptr)
        {
            return liftMotor.get()->GetCounts() / liftMotor.get()->GetCountsPerInch();
        }
    }
    return 0.0;
}
double ClimberState::GetRotateAngle() const
{
    if (m_climber != nullptr)
    {
        auto rotateMotor = m_climber->GetSecondaryMotor();
        if (rotateMotor.get() != nullptr)
        {
            return rotateMotor.get()->GetCounts() / rotateMotor.get()->GetCountsPerDegree();
        }
    }
    return 0.0;
}
