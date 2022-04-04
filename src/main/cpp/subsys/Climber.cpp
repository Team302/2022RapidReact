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
    m_reachMin(liftMotor.get()->GetRotations()),
    m_reachMax(liftMotor.get()->GetRotations()+liftMotor.get()->GetCountsPerInch()*19.25),
    m_rotateMin(rotateMotor.get()->GetRotations()),
    m_rotateMax(rotateMotor.get()->GetRotations()+rotateMotor.get()->GetCountsPerDegree()*95.0),
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
        auto currentPos = GetPrimaryPosition();
        auto atMin = false;
        auto atMax = false;
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

        if (atMin || atMax)
        {
            liftMotor.get()->GetSpeedController()->StopMotor();
        }
        else
        {
            liftMotor.get()->Set(table, liftTarget);
        }
    }
    
    auto rotateMotor = GetSecondaryMotor();
    if ( rotateMotor.get() != nullptr)
    {
        auto currentPos = GetPrimaryPosition();
        auto rotateTarget = GetSecondaryTarget();
        auto atMin = false;
        auto atMax = false;
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
        atMin = atMin || m_armBack.get()->Get();
        if (atMin || atMax)
        {
            rotateMotor.get()->GetSpeedController()->StopMotor();
        }
        else
        {
            rotateMotor.get()->Set(table, rotateTarget);
        }
        if (m_armBack.get()->Get())
        {
            rotateMotor.get()->SetSelectedSensorPosition(0.0);
            m_rotateMin = 0.0;
            m_rotateMax = rotateMotor.get()->GetCountsPerDegree()*95.0;
        }
    }

    LogData();
}


/// @brief log data to the network table if it is activated and time period has past
void Climber::LogData()
{
    Mech2IndMotors::LogData();

    auto ntName = GetNetworkTableName();
    auto table = nt::NetworkTableInstance::GetDefault().GetTable(ntName);

    Logger::GetLogger()->ToNtTable(table, "Arm Back Switch", m_armBack.get()->Get() ? "true" : "false");
}
