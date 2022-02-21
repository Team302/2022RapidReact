
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

#include <frc/DigitalInput.h>

#include <hw/DragonDigitalInput.h>
#include <hw/usages/DigitalInputUsage.h>
#include <utils/Logger.h>

using namespace frc;
using namespace std;

DragonDigitalInput::DragonDigitalInput
(
	DigitalInputUsage::DIGITAL_SENSOR_USAGE		usage,	    	// <I> - Usage of the digital input
	int 										deviceID,		// <I> - digial io ID
	bool										reversed		// <I>
) : m_digital( new DigitalInput( deviceID ) ),
	m_reversed( reversed ),
	m_type( usage )
{
}

DragonDigitalInput::~DragonDigitalInput()
{
	delete m_digital;
}

DigitalInputUsage::DIGITAL_SENSOR_USAGE DragonDigitalInput::GetType() const
{
    return m_type;
}

bool DragonDigitalInput::Get() const
{
	bool isSet = false;
	if ( m_digital != nullptr )
	{
		isSet = (m_reversed) ? !m_digital->Get() : m_digital->Get();
	}
	else
	{
        Logger::GetLogger()->LogError( string("DragonXBox::GetAxisValue"), string("DigitalInput not created") );
	}
	return isSet;
}
int  DragonDigitalInput::GetChannel() const
{
	int channel = 0;
	if ( m_digital != nullptr )
	{
		channel = m_digital->GetChannel();
	}
	return channel;
}

