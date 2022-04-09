
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

// Team 302 includes
#include <subsys/Intake.h>
#include <subsys/Mech2IndMotors.h>
#include <hw/interfaces/IDragonMotorController.h>
#include <hw/MotorData.h>

// Third Party Includes

using namespace std;

Intake::Intake
(
    MechanismTypes::MECHANISM_TYPE      type,
    string                              controlFileName,
    string                              ntName,                        
    shared_ptr<IDragonMotorController> spinMotor,
    shared_ptr<IDragonMotorController> extendMotor   
) : Mech2IndMotors(type, 
                   controlFileName, 
                   ntName, 
                   spinMotor, 
                   extendMotor)
{
    spinMotor.get()->SetFramePeriodPriority(IDragonMotorController::MOTOR_PRIORITY::LOW);
    extendMotor.get()->SetFramePeriodPriority(IDragonMotorController::MOTOR_PRIORITY::LOW);
}

bool Intake::StopIfFullyExtended() const
{
    auto motor = GetSecondaryMotor();
    if (motor.get() != nullptr)
    {
        auto fullyExtended = motor.get()->IsForwardLimitSwitchClosed();
        if (fullyExtended)
        {
            motor.get()->Set(0.0);
        }
        return fullyExtended;
    }
    return false;
}
bool Intake::StopIfRetracted() const
{
    auto motor = GetSecondaryMotor();
    if (motor.get() != nullptr)
    {
        auto fullyRetracted = motor.get()->IsReverseLimitSwitchClosed();
        /*if (!fullyRetracted)
        {
            fullyRetracted = MotorData::GetInstance()->checkIfStall(motor);
        }*/
        if (fullyRetracted)
        {
            motor.get()->Set(0.0);
        }
        return fullyRetracted;
    }
    return false;
}
