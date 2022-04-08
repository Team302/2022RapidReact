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

#include <controllers/ControlData.h>
#include <controllers/MechanismTargetData.h>
#include <hw/factories/PigeonFactory.h>
#include <states/Climber/ClimberState.h>
#include <states/Mech2MotorState.h>
#include <subsys/MechanismFactory.h>

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
    m_rotateController(new DragonPID(controlData2))
{
}

void ClimberState::Init()
{
    if (m_climber != nullptr)
    {
        auto liftMotor = m_climber->GetPrimaryMotor();
        if (liftMotor.get() != nullptr)
        {
            liftMotor.get()->SetControlMode(ControlModes::CONTROL_TYPE::PERCENT_OUTPUT);
        }

        auto rotateMotor = m_climber->GetSecondaryMotor();
        if (rotateMotor.get() != nullptr)
        {    
            rotateMotor.get()->SetControlMode(ControlModes::CONTROL_TYPE::PERCENT_OUTPUT);
        }
    }
}

void ClimberState::Run()
{
    if (m_climber != nullptr)
    {
        auto liftOutput = 0.0;
        auto rotateOutput = 0.0;

        // currently using percent output and adjusting it.   Should probably switch to 
        // SetVoltage() calls.
        auto liftMotor = m_climber->GetPrimaryMotor();
        if (liftMotor.get() != nullptr)
        {
            auto mc = liftMotor.get()->GetSpeedController();
            if (mc.get() != nullptr)
            {
                liftOutput = m_liftController->Calculate(mc.get()->Get(), GetLiftHeight(), m_liftTarget);
                liftOutput = clamp(liftOutput, -1.0, 1.0);
                if (liftOutput > 0.05 && liftOutput < 0.2 )
                {
                    liftOutput = 0.35;
                }
                else if (liftOutput < -0.05 && liftOutput > -0.2 )
                {
                    liftOutput = -0.35;
                }
            }
        }

        auto rotateMotor = m_climber->GetSecondaryMotor();
        if (rotateMotor.get() != nullptr)
        {
            auto mc = rotateMotor.get()->GetSpeedController();
            if (mc.get() != nullptr)
            {
                rotateOutput = m_rotateController->Calculate(mc.get()->Get(), GetRotateAngle(), m_rotateTarget);
                rotateOutput = clamp(rotateOutput, -1.0, 1.0);
            }
        }
        m_climber->UpdateTargets(liftOutput, rotateOutput);
        m_climber->Update();
    }
}
bool ClimberState::AtTarget() const
{
    auto deltaHeight = GetLiftHeight()-m_liftTarget;
    auto deltaAngle = GetRotateAngle()-m_rotateTarget;
    auto pigeon = PigeonFactory::GetFactory()->GetCenterPigeon();
    auto deltaPitch = pigeon != nullptr ? pigeon->GetPitch() - m_robotPitch : 0.0;
    return abs(deltaHeight) < 0.25 &&
           abs(deltaAngle) < 2.0 &&
           abs(deltaPitch) < 2.0;
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
