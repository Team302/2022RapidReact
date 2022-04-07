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
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>

#include <units/voltage.h>

// Team 302 includes
#include <hw/DragonAnalogInput.h>
#include <hw/DragonDigitalInput.h>
#include <hw/interfaces/IDragonMotorController.h>
#include <hw/MotorData.h>
#include <subsys/Climber.h>
#include <utils/Logger.h>

// Third Party Includes
using namespace std;

Climber::Climber
(
    shared_ptr<IDragonMotorController>      liftMotor,
    shared_ptr<IDragonMotorController>      rotateMotor,
    std::shared_ptr<DragonDigitalInput>     armBackSw
) : Mech2IndMotors( MechanismTypes::MECHANISM_TYPE::CLIMBER,  string("climber.xml"),  string("ClimberNT"), liftMotor, rotateMotor ),
    m_reachMin(0.0),
    m_reachMax(m_reachMin+19.25),
    m_rotateMin(0.0),
    m_rotateMax(m_rotateMin+95.0),
    m_armBack(armBackSw)
{
    liftMotor.get()->SetFramePeriodPriority(IDragonMotorController::MOTOR_PRIORITY::LOW);
    rotateMotor.get()->SetFramePeriodPriority(IDragonMotorController::MOTOR_PRIORITY::LOW);

    liftMotor.get()->SetSelectedSensorPosition(0.0);
    rotateMotor.get()->SetSelectedSensorPosition(0.0);
}


/// @brief update the output to the mechanism using the current controller and target value(s)
/// @return void 
void Climber::Update()
{
    auto ntName = GetNetworkTableName();
    auto table = nt::NetworkTableInstance::GetDefault().GetTable(ntName);

    auto liftMotor = GetPrimaryMotor();
    if ( liftMotor.get() != nullptr )
    {
        auto atMinReach = IsAtMinReach(liftMotor);
        auto atMaxReach = IsAtMaxReach(liftMotor);

        if ((!atMaxReach && !atMinReach) ||
            (atMaxReach && m_liftTarget < 0.0) ||
            (atMinReach && m_liftTarget > 0.0))
        {
            liftMotor.get()->SetVoltage(units::volt_t(m_liftTarget*10.0));
        }
        else
        {
            liftMotor.get()->SetVoltage(units::volt_t(0.0));
        }
   }
    auto rotateMotor = GetSecondaryMotor();
    if ( rotateMotor.get() != nullptr)
    {
        auto atMinRot = IsAtMinRotation(rotateMotor);
        auto atMaxRot = IsAtMaxRotation(rotateMotor);

        if ((!atMaxRot && !atMinRot) ||
            (atMaxRot && m_rotateTarget < 0.0) ||
            (atMinRot && m_rotateTarget > 0.0))
        {
            rotateMotor.get()->SetVoltage(units::volt_t(m_rotateTarget*10.0));
        }
        else
        {
            rotateMotor.get()->SetVoltage(units::volt_t(0.0));
        }
    }

    LogData();
}

bool Climber::IsAtMaxReach
(
    std::shared_ptr<IDragonMotorController> liftMotor
) const
{
    auto currentHeight = liftMotor.get()->GetInches();
    auto atMax = currentHeight >= m_reachMax;
    atMax = !atMax ? liftMotor.get()->IsForwardLimitSwitchClosed() : atMax;
    return atMax;

}
bool Climber::IsAtMinReach
(
    std::shared_ptr<IDragonMotorController> liftMotor
) const
{
    auto currentHeight = liftMotor.get()->GetInches();
    auto atMin = currentHeight <= m_reachMin;
    atMin = !atMin ? liftMotor.get()->IsReverseLimitSwitchClosed() : atMin;
    return atMin;
}
bool Climber::IsAtMaxRotation
(
    std::shared_ptr<IDragonMotorController> rotateMotor
) const
{
    auto currentAngle = rotateMotor.get()->GetInches();
    auto atMin = currentAngle <= m_rotateMin;
    atMin = atMin || m_armBack.get()->Get();
    return atMin;
}
bool Climber::IsAtMinRotation
(
    std::shared_ptr<IDragonMotorController> rotateMotor
) const
{
    auto currentAngle = rotateMotor.get()->GetInches();
    auto atMax = currentAngle >= m_rotateMax;
    return atMax;
}

bool Climber::IsLiftStalled() const
{
    auto liftMotor = GetPrimaryMotor();
    return liftMotor.get() != nullptr ? MotorData::GetInstance()->checkIfStall(liftMotor) : false;
}
bool Climber::IsRotateStalled() const
{
    auto rotateMotor = GetSecondaryMotor();
    return rotateMotor.get() != nullptr ? MotorData::GetInstance()->checkIfStall(rotateMotor) : false;
}



/// @brief log data to the network table if it is activated and time period has past
void Climber::LogData()
{
    Mech2IndMotors::LogData();

    auto ntName = GetNetworkTableName();
    auto table = nt::NetworkTableInstance::GetDefault().GetTable(ntName);

    Logger::GetLogger()->ToNtTable(table, string("Reach - Min"), m_reachMin);
    Logger::GetLogger()->ToNtTable(table, string("Reach - Max"), m_reachMax);
    Logger::GetLogger()->ToNtTable(table, string("Reach - Target"), m_liftTarget);

    auto liftMotor = GetPrimaryMotor();
    if (liftMotor.get() != nullptr)
    {
        Logger::GetLogger()->ToNtTable(table, string("Reach - Current"), liftMotor.get()->GetInches());
        Logger::GetLogger()->ToNtTable(table, string("Reach - Min Switch"), liftMotor.get()->IsReverseLimitSwitchClosed() ? "true" : "false");
        Logger::GetLogger()->ToNtTable(table, string("Reach - Max Switch"), liftMotor.get()->IsForwardLimitSwitchClosed() ? "true" : "false");
    }

    Logger::GetLogger()->ToNtTable(table, string("Rotate - Min"), m_rotateMin);
    Logger::GetLogger()->ToNtTable(table, string("Rotate - Max"), m_rotateMax);
    
    auto rotateMotor = GetSecondaryMotor();
    if (rotateMotor.get() != nullptr)
    {
        Logger::GetLogger()->ToNtTable(table, string("Rotate - Current"), rotateMotor.get()->GetDegrees());
    }
    Logger::GetLogger()->ToNtTable(table, string("Rotate - Target"), m_rotateTarget);

    Logger::GetLogger()->ToNtTable(table, string("Rotate - Arm Back Switch"), m_armBack.get()->Get() ? "true" : "false");
}

void Climber::SetTargetAngle
(
    double angle
)
{
    m_rotateTarget = angle;
}
void Climber::SetTargetHeight
(
    double height
)
{
    m_liftTarget = height;
}

