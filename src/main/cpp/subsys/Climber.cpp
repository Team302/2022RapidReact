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
    m_reachMin(GetPositionInInches(liftMotor)),
    m_reachMax(m_reachMin+19.25),
    m_rotateMin(GetPositionInDegrees(rotateMotor)),
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
        auto liftTarget = GetPrimaryTarget();
        auto currentPos = GetPositionInInches(liftMotor);
        auto atMinReach = IsAtMinReach(liftMotor, currentPos);
        auto atMaxReach = IsAtMaxReach(liftMotor, currentPos);

        /** **/
        if ((atMinReach && liftTarget <= currentPos) || (atMaxReach && liftTarget >= currentPos))
        {
            liftMotor.get()->GetSpeedController()->StopMotor();
        }
        else
        {
            liftMotor.get()->Set(table, liftTarget);
        }
        /** **/    
   }
    auto rotateMotor = GetSecondaryMotor();
    if ( rotateMotor.get() != nullptr)
    {
        auto rotateTarget = GetSecondaryTarget();
        auto currentPos = GetPositionInDegrees(rotateMotor);
        auto atMinRot = IsAtMinRotation(rotateMotor, currentPos);
        auto atMaxRot = IsAtMaxRotation(rotateMotor, currentPos);
        /** **/
        if ((atMinRot && rotateTarget <= currentPos) || (atMaxRot && rotateTarget >= currentPos))
        {
            rotateMotor.get()->GetSpeedController()->StopMotor();
        }
        else
        {
            rotateMotor.get()->Set(table, rotateTarget);
        }
    }

    LogData();
}

bool Climber::IsAtMaxReach
(
    std::shared_ptr<IDragonMotorController> liftMotor,
    double                                  currentHeight
) const
{
    auto atMax = currentHeight >= m_reachMax;
    atMax = !atMax ? liftMotor.get()->IsForwardLimitSwitchClosed() : atMax;
    return atMax;

}
bool Climber::IsAtMinReach
(
    std::shared_ptr<IDragonMotorController> liftMotor,
    double                                  currentHeight
) const
{
    auto atMin = currentHeight <= m_reachMin;
    atMin = !atMin ? liftMotor.get()->IsReverseLimitSwitchClosed() : atMin;
    return atMin;
}
bool Climber::IsAtMinRotation
(
    std::shared_ptr<IDragonMotorController> liftMotor,
    double                                  currentAngle
) const
{
    auto atMin = currentAngle <= m_rotateMin;
    atMin = m_armBack.get()->Get();
    return atMin;
}
bool Climber::IsAtMaxRotation
(
    std::shared_ptr<IDragonMotorController> liftMotor,
    double                                  currentAngle
) const
{
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

    Logger::GetLogger()->ToNtTable(table, string("Reach - Min"), GetMinReach());
    Logger::GetLogger()->ToNtTable(table, string("Reach - Max"), GetMaxReach());
    Logger::GetLogger()->ToNtTable(table, string("Reach - Current"), GetPositionInInches(GetPrimaryMotor()));
    Logger::GetLogger()->ToNtTable(table, string("Reach - Target"), GetPrimaryTarget());

    auto liftMotor = GetPrimaryMotor();
    if (liftMotor.get() != nullptr)
    {
        Logger::GetLogger()->ToNtTable(table, string("Reach - Min Switch"), liftMotor.get()->IsReverseLimitSwitchClosed() ? "true" : "false");
        Logger::GetLogger()->ToNtTable(table, string("Reach - Max Switch"), liftMotor.get()->IsForwardLimitSwitchClosed() ? "true" : "false");
    }

    Logger::GetLogger()->ToNtTable(table, string("Rotate - Min"), GetMinRotate());
    Logger::GetLogger()->ToNtTable(table, string("Rotate - Max"), GetMaxRotate());
    Logger::GetLogger()->ToNtTable(table, string("Rotate - Current"), GetPositionInDegrees(GetSecondaryMotor()));
    Logger::GetLogger()->ToNtTable(table, string("Rotate - Target"), GetSecondaryTarget());

    Logger::GetLogger()->ToNtTable(table, string("Rotate - Arm Back Switch"), m_armBack.get()->Get() ? "true" : "false");
}
double Climber::GetPositionInInches
(
    std::shared_ptr<IDragonMotorController> motor
)
{
    if (motor.get() != nullptr)
    {
        auto rot = motor.get()->GetRotations();
        auto countsPerRot = motor.get()->GetCountsPerRev();
        auto countsPerInch = motor.get()->GetCountsPerInch();
        return ((rot * countsPerRot)/ countsPerInch);
    }
    return 0.0;
}
double Climber::GetPositionInDegrees
(
    std::shared_ptr<IDragonMotorController> motor
)
{
    if (motor.get() != nullptr)
    {
        auto rot = motor.get()->GetRotations();
        auto countsPerRot = motor.get()->GetCountsPerRev();
        auto countsPerDegree = motor.get()->GetCountsPerDegree();
        return ((rot * countsPerRot)/ countsPerDegree);
    }
    return 0.0;

}
