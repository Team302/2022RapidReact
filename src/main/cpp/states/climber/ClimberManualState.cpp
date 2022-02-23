
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
#include <memory>

// FRC includes

// Team 302 includes
#include <controllers/ControlData.h>
#include <controllers/MechanismTargetData.h>
#include <gamepad/TeleopControl.h>
#include <states/climber/ClimberManualState.h>
#include <states/IState.h>
#include <subsys/interfaces/IMech2IndMotors.h>
#include <utils/Logger.h>


// Third Party Includes

using namespace std;

/// @class ClimberManualState
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
ClimberManualState::ClimberManualState
(
    ControlData*                    controlDataUpDown,
    ControlData*                    controlDataRotate,
    double                          maxSpeedUpDown,
    double                          maxSpeedRotate,
    Climber*                        climber
) : IState(),
    m_climber(climber),
    m_controller(TeleopControl::GetInstance()),
    m_controlDataUpDown(controlDataUpDown),
    m_controlDataRotate(controlDataRotate),
    m_upDownMax(maxSpeedUpDown),
    m_rotateMax(maxSpeedRotate)
{
    if (climber == nullptr)
    {
        Logger::GetLogger()->LogError(string("ClimberManualState::ClimberManualState"), string("no climber"));
    }    
    
    if (controlDataUpDown == nullptr)
    {
        Logger::GetLogger()->LogError(string("Mech2MotorState::Mech2MotorState"), string("no control data"));
    }    
    else if (controlDataRotate == nullptr)
    {
        Logger::GetLogger()->LogError(string("Mech2MotorState::Mech2MotorState"), string("no control2 data"));
    }
}

void ClimberManualState::Init()
{
    if (m_climber != nullptr)
    {
        m_climber->SetControlConstants(0, m_controlDataUpDown);
        m_climber->SetSecondaryControlConstants(0, m_controlDataRotate);
        m_climber->UpdateTargets(m_upDownMax, m_rotateMax);
    }
}


void ClimberManualState::Run()           
{
    if (m_climber != nullptr && m_controller != nullptr)
    {
        auto armDownPercent = m_controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::CLIMBER_MAN_DOWN);
        auto armUpPercent   = m_controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::CLIMBER_MAN_UP);
        auto armRotatePercent = m_controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::CLIMBER_MAN_ROTATE);

        auto upDownPercent = armUpPercent - armDownPercent;
        auto upDowntarget = upDownPercent * m_upDownMax;
        auto rotateTarget = armRotatePercent * m_rotateMax; 
        m_climber->UpdateTargets(upDowntarget, rotateTarget);
        m_climber->Update();
        m_climber->LogData();
    }
}

bool ClimberManualState::AtTarget() const
{
    return true;
}

