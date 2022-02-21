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
#include <controllers/ControlData.h>
#include <hw/DragonSolenoid.h>
#include <hw/interfaces/IDragonMotorController.h>
#include <subsys/Mech1IndMotor.h>
#include <subsys/Mech1IndMotorSolenoid.h>
#include <subsys/Mech1Solenoid.h>
#include <subsys/interfaces/IMech1IndMotor.h>
#include <subsys/interfaces/IMech1IndMotorSolenoid.h>
#include <subsys/interfaces/IMech1Solenoid.h>

// Third Party Includes
#include <units/time.h>

using namespace frc;
using namespace std;

/// @brief Create a generic mechanism wiht 1 independent motor 
/// @param [in] MechanismTypes::MECHANISM_TYPE the type of mechansim
/// @param [in] std::string the name of the file that will set control parameters for this mechanism
/// @param [in] std::string the name of the network table for logging information
/// @param [in] std::shared_ptr<IDragonMotorController> motor controller used by this mechanism
Mech1IndMotorSolenoid::Mech1IndMotorSolenoid

(
    MechanismTypes::MECHANISM_TYPE              type,
    string                                      controlFileName,
    string                                      networkTableName,
    shared_ptr<IDragonMotorController>          motorController,
    shared_ptr<DragonSolenoid>                  solenoid
) : m_motorMech(new Mech1IndMotor(type, controlFileName, networkTableName, motorController)),
    m_solenoidMech(new Mech1Solenoid(type, controlFileName, networkTableName, solenoid))
{
}

/// @brief          Indicates the type of mechanism this is
/// @return         MechanismTypes::MECHANISM_TYPE
MechanismTypes::MECHANISM_TYPE Mech1IndMotorSolenoid::GetType() const 
{
    if (m_motorMech != nullptr)
    {
        return m_motorMech->GetType();
    }
    else if (m_solenoidMech != nullptr)
    {
        return m_solenoidMech->GetType();
    }
    return MechanismTypes::MECHANISM_TYPE::UNKNOWN_MECHANISM;
}

/// @brief indicate the file used to get the control parameters from
/// @return std::string the name of the file 
string Mech1IndMotorSolenoid::GetControlFileName() const 
{
    if (m_motorMech != nullptr)
    {
        return m_motorMech->GetControlFileName();
    }
    else if (m_solenoidMech != nullptr)
    {
        return m_solenoidMech->GetControlFileName();
    }
    return string();
}


/// @brief indicate the network table name used to for logging parameters
/// @return std::string the name of the network table 
string Mech1IndMotorSolenoid::GetNetworkTableName() const 
{
    if (m_motorMech != nullptr)
    {
        return m_motorMech->GetNetworkTableName();
    }
    else if (m_solenoidMech != nullptr)
    {
        return m_solenoidMech->GetNetworkTableName();
    }
    return string();}

/// @brief log data to the network table if it is activated and time period has past
void Mech1IndMotorSolenoid::LogData()
{
    if (m_motorMech != nullptr)
    {
        m_motorMech->LogData();
    }
    if (m_solenoidMech != nullptr)
    {
        m_solenoidMech->LogData();
    }
}

void Mech1IndMotorSolenoid::Update()
{
    if (m_motorMech != nullptr)
    {
        m_motorMech->Update();
    }
}

void Mech1IndMotorSolenoid::UpdateTarget
(
    double  target
)
{
    if (m_motorMech != nullptr)
    {
        m_motorMech->UpdateTarget(target);
    }
}


double Mech1IndMotorSolenoid::GetPosition() const

{
    if (m_motorMech != nullptr)
    {
        return m_motorMech->GetPosition();
    }
    return 0.0;
}



double Mech1IndMotorSolenoid::GetSpeed() const

{
    if (m_motorMech != nullptr)
    {
        return m_motorMech->GetSpeed();
    }
    return 0.0;
}


/// @brief  Set the control constants (e.g. PIDF values).
/// @param [in] ControlData* pid:  the control constants
/// @return void
void Mech1IndMotorSolenoid::SetControlConstants
(
    int                                         slot,
    ControlData*                                pid                 
)
{
    if (m_motorMech != nullptr)
    {
        return m_motorMech->SetControlConstants(slot, pid);
    }
}


/// @brief      Activate/deactivate pneumatic solenoid
/// @param [in] bool - true == extend, false == retract
/// @return     void 
void Mech1IndMotorSolenoid::ActivateSolenoid
(
    bool activate
)
{
    if (m_solenoidMech != nullptr)
    {
        m_solenoidMech->ActivateSolenoid(activate);
    }
}


/// @brief      Check if the pneumatic solenoid is activated
/// @return     bool - true == extended, false == retracted
bool Mech1IndMotorSolenoid::IsSolenoidActivated() const
{
    if (m_solenoidMech != nullptr)
    {
        return m_solenoidMech->IsSolenoidActivated();
    }
    return false;
}

IMech1IndMotor* Mech1IndMotorSolenoid::Get1IndMotorMech() const
{
    return m_motorMech;
}
IMech1Solenoid* Mech1IndMotorSolenoid::GetSolenoidMech() const
{
    return m_solenoidMech;
}

Mech1IndMotorSolenoid::~Mech1IndMotorSolenoid()
{
    delete m_motorMech;
    delete m_solenoidMech;

    m_motorMech = nullptr;
    m_solenoidMech = nullptr;
}

