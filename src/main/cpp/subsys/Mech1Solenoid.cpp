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

#include <subsys/Mech1Solenoid.h>
#include <subsys/interfaces/IMech1Solenoid.h>
#include <hw/DragonSolenoid.h>
#include <utils/Logger.h>

// Third Party Includes

using namespace std;

/// @brief Create a generic mechanism wiht 1 solenoid 
/// @param [in] MechanismTypes::MECHANISM_TYPE the type of mechansim
/// @param [in] std::string the name of the file that will set control parameters for this mechanism
/// @param [in] std::string the name of the network table for logging information
/// @param [in] std::shared_ptr<DragonSolenoid> solenoid used by this mechanism
Mech1Solenoid::Mech1Solenoid
(
    MechanismTypes::MECHANISM_TYPE          type,
    string                                  controlFileName,
    string                                  networkTableName,
    shared_ptr<DragonSolenoid>              solenoid
) : IMech1Solenoid(),
    m_solenoid( solenoid ),
    m_type(type),
    m_controlFile(controlFileName),
    m_ntName(networkTableName)

{
    if (m_solenoid.get() == nullptr )
    {
        Logger::GetLogger()->LogError( Logger::LOGGER_LEVEL::ERROR_ONCE, string( "Mech1Solenoid constructor" ), string( "solenoid is nullptr" ) );
    }
}


/// @brief          Indicates the type of mechanism this is
/// @return         MechanismTypes::MECHANISM_TYPE
MechanismTypes::MECHANISM_TYPE Mech1Solenoid::GetType() const 
{
    return m_type;
}

/// @brief indicate the file used to get the control parameters from
/// @return std::string the name of the file 
std::string Mech1Solenoid::GetControlFileName() const 
{
    return m_controlFile;
}


/// @brief indicate the network table name used to for logging parameters
/// @return std::string the name of the network table 
std::string Mech1Solenoid::GetNetworkTableName() const 
{
    return m_ntName;
}

/// @brief log data to the network table if it is activated and time period has past
void Mech1Solenoid::LogData()
{
}


/// @brief      Activate/deactivate pneumatic solenoid
/// @param [in] bool - true == extend, false == retract
/// @return     void 
void Mech1Solenoid::ActivateSolenoid
(
    bool activate
)
{
    if ( m_solenoid.get() != nullptr )
    {
        m_solenoid.get()->Set( activate );
    }
}


/// @brief      Check if the pneumatic solenoid is activated
/// @return     bool - true == extended, false == retracted
bool Mech1Solenoid::IsSolenoidActivated() const
{
    return  ( m_solenoid.get() != nullptr ) ? m_solenoid.get()->Get() : false;

}







