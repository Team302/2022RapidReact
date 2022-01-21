
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
#include <string.h>

// FRC includes

#include <auton/FieldMeasurement.h>
#include <auton/FieldDefn.h>
#include <utils/Logger.h>

// Third Party Includes
#include <pugixml/pugixml.hpp>

using namespace std;
using namespace pugi;

//-----------------------------------------------------------------------
// Method:      ParseXML
// Description: Parse a Field XML element and create an IFieldMeasurement
//              from its definition.
//
// Returns:     void
//-----------------------------------------------------------------------
void FieldDefn::ParseXML
(
    xml_node      fieldNode
)
{
    // initialize attributes
	FieldMeasurement* m_measurements = FieldMeasurement::GetFieldMeasurement();
	FieldMeasurement::Measurement type = FieldMeasurement::UNKNOWN_FIELD;
	float xLoc = 0.0;
	float yLoc = 0.0;

    bool hasError = false;

    // Parse/validate xml
    for (xml_attribute attr = fieldNode.first_attribute(); attr; attr = attr.next_attribute())
    {
        if ( strcmp( attr.name(), "type" ) == 0 )
        {
            int iVal = attr.as_int();
            switch ( iVal )
            {
                case FieldMeasurement::BLUE_SWITCH_RIGHT:
                    type = FieldMeasurement::BLUE_SWITCH_RIGHT;
                    break;

                case FieldMeasurement::BLUE_SWITCH_LEFT:
                    type = FieldMeasurement::BLUE_SWITCH_LEFT;
                    break;

                case FieldMeasurement::BLUE_SCALE_RIGHT:
                    type = FieldMeasurement::BLUE_SCALE_RIGHT;
                    break;

                case FieldMeasurement::BLUE_SCALE_LEFT:
                    type = FieldMeasurement::BLUE_SCALE_LEFT;
                    break;

                case FieldMeasurement::RED_SWITCH_RIGHT:
                    type = FieldMeasurement::RED_SWITCH_RIGHT;
                    break;

                case FieldMeasurement::RED_SWITCH_LEFT:
                    type = FieldMeasurement::RED_SWITCH_LEFT;
                    break;

                case FieldMeasurement::RED_SCALE_RIGHT:
                    type = FieldMeasurement::RED_SCALE_RIGHT;
                    break;

                case FieldMeasurement::RED_SCALE_LEFT:
                    type = FieldMeasurement::RED_SCALE_LEFT;
                    break;

                default:
                    Logger::GetLogger()->LogError( "FieldDefn::ParseXML", "unknown field piece type" );
                    hasError = true;
                    break;
            }
        }
        else if ( strcmp( attr.name(), "xloc" ) == 0 )
        {
        	xLoc = attr.as_float();
        }
        else if ( strcmp( attr.name(), "yloc" ) == 0 )
        {
        	yLoc = attr.as_float();
        }
        else
        {
            string msg = "invalid attribute";
            msg += attr.name();
            Logger::GetLogger()->LogError( "FieldDefn::ParseXML", msg );
            hasError = true;
        }
    }

    m_measurements->AddLocation(type, xLoc, yLoc);

    if ( !hasError )
    {

    }
}
