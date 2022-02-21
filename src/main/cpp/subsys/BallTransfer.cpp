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
#include <subsys/BallTransfer.h>
#include <subsys/Mech2IndMotors.h>
#include <hw/interfaces/IDragonMotorController.h>
#include <hw/DragonDigitalInput.h>
#include <subsys/MechanismFactory.h>



// Third Party Includes

using namespace std;

BallTransfer::BallTransfer
(
    string networkTableName,
    string controlFileName,
    shared_ptr<IDragonMotorController> spinMotor, 
    shared_ptr<IDragonMotorController> liftMotor,
    shared_ptr<DragonDigitalInput>     ballPresentSw,
    shared_ptr<DragonDigitalInput>     liftForwardSw
) : Mech2IndMotors(MechanismTypes::MECHANISM_TYPE::BALL_TRANSFER, 
                   controlFileName, 
                   networkTableName, 
                   spinMotor, 
                   liftMotor),
    m_ballPresentSw(ballPresentSw),
    m_liftForwardSw(liftForwardSw)
{
    spinMotor.get()->SetFramePeriodPriority(IDragonMotorController::MOTOR_PRIORITY::LOW);
    liftMotor.get()->SetFramePeriodPriority(IDragonMotorController::MOTOR_PRIORITY::MEDIUM);
}

bool BallTransfer::IsBallPresent() const
{
    if (m_ballPresentSw.get() != nullptr)
    {
        return m_ballPresentSw.get()->Get();
    }
    return false;
}

bool BallTransfer::IsLiftForward() const
{
    if (m_liftForwardSw.get() != nullptr)
    {
        return m_liftForwardSw.get()->Get();
    }
    return false;
}