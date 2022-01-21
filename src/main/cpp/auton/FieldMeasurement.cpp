
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


#include <iostream>
#include <auton/FieldMeasurement.h>

FieldMeasurement* FieldMeasurement::m_fieldMeasurement = nullptr;

FieldMeasurement* FieldMeasurement::GetFieldMeasurement()
{
	if ( FieldMeasurement::m_fieldMeasurement == nullptr )
	{
		FieldMeasurement::m_fieldMeasurement = new FieldMeasurement();
	}
	return FieldMeasurement::m_fieldMeasurement;
}

FieldMeasurement::FieldMeasurement() : m_xLoc(),
									   m_yLoc()
{
	for ( int inx=0; inx<FieldMeasurement::MAX_POSITIONS; ++inx )
	{
		m_xLoc[inx] = 0.0;
		m_yLoc[inx] = 0.0;
	}

}

float FieldMeasurement::GetXLoc(Measurement location)
{
	float value = 0.0;
	if ( location > -1 && location < FieldMeasurement::MAX_POSITIONS )
	{
		value = m_xLoc[ location ];
	}
	else
	{
		printf( "==>> invalid XLoc requested \n");
	}
	return value;
}

float FieldMeasurement::GetYLoc(Measurement location)
{
	float value = 0.0;
	if ( location > -1 && location < FieldMeasurement::MAX_POSITIONS )
	{
		value = m_yLoc[ location ];
	}
	else
	{
		printf( "==>> invalid YLoc requested \n");
	}
	return value;
}

void FieldMeasurement::AddLocation
(
		Measurement location,
		float X,
		float Y
)
{
	if ( location > -1 && location < FieldMeasurement::MAX_POSITIONS )
	{
		m_xLoc[ location ] = X;
	}
	else
	{
		printf( "==>> invalid XLoc requested \n");
	}
}

float FieldMeasurement::GetKeyPoint( Measurement keyPointLocation )
{
	float keyPoint = 0.0;

	switch( keyPointLocation )
	{
		case BLUE_SCALE_LEFT:
			keyPoint = GetYLoc( keyPointLocation ) - distances[0];
			break;
		case BLUE_SCALE_RIGHT:
			keyPoint = GetYLoc( keyPointLocation ) - distances[1];
			break;
		case RED_SCALE_LEFT:
			keyPoint = GetYLoc( keyPointLocation ) - distances[0];
			break;
		case RED_SCALE_RIGHT:
			keyPoint = GetYLoc( keyPointLocation ) - distances[1];
			break;

		default:
			printf( "==>> FieldMeasurement::GetKeyPoint: unknown keypoint \n" );
			keyPoint = 0.0;
			break;
	}

	return keyPoint;
}
