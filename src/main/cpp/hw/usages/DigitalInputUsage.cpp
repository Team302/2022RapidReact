
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
#include <map>
#include <memory>
#include <string>

// FRC includes

// Team 302 includes
#include <hw/usages/DigitalInputUsage.h>

// Third Party Includes

using namespace std;

DigitalInputUsage* DigitalInputUsage::m_instance = nullptr;
DigitalInputUsage* DigitalInputUsage::GetInstance()
{
    if ( m_instance == nullptr )
    {
        m_instance = new DigitalInputUsage();
    }
    return m_instance;
}

DigitalInputUsage::DigitalInputUsage()
{
    m_usageMap["INTAKE_OUT"] = DigitalInputUsage::DIGITAL_SENSOR_USAGE::INTAKE_OUT;
    m_usageMap["INTAKE_IN"] = DigitalInputUsage::DIGITAL_SENSOR_USAGE::INTAKE_IN;
    m_usageMap["BALL_PRESENT"] = DigitalInputUsage::DIGITAL_SENSOR_USAGE::BALL_PRESENT;
    m_usageMap["BALL_TRANSFER_FORWARD"] = DigitalInputUsage::DIGITAL_SENSOR_USAGE::BALL_TRANSFER_FORWARD;
    m_usageMap["BALL_TRANSFER_BACK"] = DigitalInputUsage::DIGITAL_SENSOR_USAGE::BALL_TRANSFER_BACK;
    m_usageMap["SHOOTER_HOOD_MIN"] = DigitalInputUsage::DIGITAL_SENSOR_USAGE::SHOOTER_HOOD_MIN;
    m_usageMap["SHOOTER_HOOD_MAX"] = DigitalInputUsage::DIGITAL_SENSOR_USAGE::SHOOTER_HOOD_MAX;
    m_usageMap["CLIMBER_BACK"] = DigitalInputUsage::DIGITAL_SENSOR_USAGE::CLIMBER_BACK;
    m_usageMap["CLIMBER_FORWARD"] = DigitalInputUsage::DIGITAL_SENSOR_USAGE::CLIMBER_FORWARD;
;
}

DigitalInputUsage::~DigitalInputUsage()
{
    m_usageMap.clear();
}

DigitalInputUsage::DIGITAL_SENSOR_USAGE DigitalInputUsage::GetUsage
(
    string              usageString
)
{
    return m_usageMap.find(usageString)->second;
}

