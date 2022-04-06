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
        auto atMin = currentPos <= m_reachMin;
        atMin = !atMin ? liftMotor.get()->IsReverseLimitSwitchClosed() : atMin;
        auto atMax = currentPos >= m_reachMax;
        atMax = !atMax ? liftMotor.get()->IsForwardLimitSwitchClosed() : atMax;
        /**
        auto mode = liftMotor.get()->GetControlMode();
        if (currentPos <= m_reachMin)
        {
            switch (mode)
            {
                case ControlModes::PERCENT_OUTPUT:
                    [[fallthrough]];
                case ControlModes::POSITION_INCH:
                    [[fallthrough]];
                case ControlModes::VELOCITY_INCH:
                  if (liftTarget < 0.0 )
                   {
                       liftTarget = 0.0;
                       atMin = true;
                   }
                   break;

                default:
                    break;
            }
        }
        if (currentPos >= m_reachMax)
        {
            switch (mode)
            {
                case ControlModes::PERCENT_OUTPUT:
                    [[fallthrough]];
                case ControlModes::POSITION_INCH:
                    [[fallthrough]];
                case ControlModes::VELOCITY_INCH:
                  if (liftTarget > 0.0 )
                   {
                       liftTarget = 0.0;
                       atMax = true;
                   }
                   break;

                default:
                    break;
            }
        }
        **/

        Logger::GetLogger()->ToNtTable(table, string("lift at min"), to_string(atMin));
        Logger::GetLogger()->ToNtTable(table, string("lift at max"), to_string(atMax));
        Logger::GetLogger()->ToNtTable(table, string("lift target"), liftTarget);

        /** **/
        if ((atMin && liftTarget <= currentPos) || (atMax && liftTarget >= currentPos))
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
        auto atMin = currentPos <= m_rotateMin;
        auto atMax = currentPos >= m_rotateMax;
        /**
        auto mode = liftMotor.get()->GetControlMode();
        if (currentPos <= m_rotateMin)
        {
            switch (mode)
            {
                case ControlModes::PERCENT_OUTPUT:
                    [[fallthrough]];
                case ControlModes::POSITION_INCH:
                    [[fallthrough]];
                case ControlModes::VELOCITY_INCH:
                  if (rotateTarget < 0.0 )
                   {
                       rotateTarget = 0.0;
                       atMin = true;
                   }
                   break;

                default:
                    break;
            }
        }
        if (currentPos >= m_reachMax)
        {
            switch (mode)
            {
                case ControlModes::PERCENT_OUTPUT:
                    [[fallthrough]];
                case ControlModes::POSITION_INCH:
                    [[fallthrough]];
                case ControlModes::VELOCITY_INCH:
                  if (rotateTarget > 0.0 )
                   {
                       rotateTarget = 0.0;
                       atMax = true;
                   }
                   break;

                default:
                    break;
            }
        }
        **/
        atMin = atMin || m_armBack.get()->Get();

        auto ntName = GetNetworkTableName();
        auto table = nt::NetworkTableInstance::GetDefault().GetTable(ntName);

        Logger::GetLogger()->ToNtTable(table, string("rotate at min"), to_string(atMin));
        Logger::GetLogger()->ToNtTable(table, string("rotate at max"), to_string(atMax));
        Logger::GetLogger()->ToNtTable(table, string("rotate target"), rotateTarget);

        /** **/
        if ((atMin && rotateTarget <= currentPos) || (atMax && rotateTarget >= currentPos))
        {
            rotateMotor.get()->GetSpeedController()->StopMotor();
        }
        else
        {
            rotateMotor.get()->Set(table, rotateTarget);
        }
        /** 
        if (m_armBack.get()->Get())
        {
            rotateMotor.get()->SetSelectedSensorPosition(0.0);
            m_rotateMin = 0.0;
            m_rotateMax = rotateMotor.get()->GetCountsPerDegree()*95.0;
        }
        **/
    }

    LogData();
}
bool Climber::IsLiftStalled() const
{
    auto liftMotor = GetPrimaryMotor();
    if (liftMotor.get() != nullptr)
    {
        auto motorType = liftMotor.get()->GetMotorType();
        return MotorData::GetInstance()->checkIfStall(liftMotor);
    }
    return false;
}
bool Climber::IsRotateStalled() const
{
    auto rotateMotor = GetSecondaryMotor();
    if (rotateMotor.get() != nullptr)
    {
        auto motorType = rotateMotor.get()->GetMotorType();
        return MotorData::GetInstance()->checkIfStall(rotateMotor);
    }
    return false;
}



/// @brief log data to the network table if it is activated and time period has past
void Climber::LogData()
{
    Mech2IndMotors::LogData();

    auto ntName = GetNetworkTableName();
    auto table = nt::NetworkTableInstance::GetDefault().GetTable(ntName);

    Logger::GetLogger()->ToNtTable(table, string("Min Reach"), GetMinReach());
    Logger::GetLogger()->ToNtTable(table, string("Max Reach"), GetMaxReach());
    Logger::GetLogger()->ToNtTable(table, string("Current Reach"), GetPrimaryMotor().get()->GetCounts());

    Logger::GetLogger()->ToNtTable(table, string("Min Rotate"), GetMinRotate());
    Logger::GetLogger()->ToNtTable(table, string("Max Rotate"), GetMaxRotate());
    Logger::GetLogger()->ToNtTable(table, string("Current Rotate"), GetSecondaryMotor().get()->GetCounts());

    Logger::GetLogger()->ToNtTable(table, string("Arm Back Switch"), m_armBack.get()->Get() ? "true" : "false");
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
