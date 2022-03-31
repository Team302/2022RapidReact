
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
   
// --------------------------------------------------------------------------------------------
// AnalogInputDefn.h
// --------------------------------------------------------------------------------------------
//
// Description: Create an AnalogInput from an XML definition
//
// 	<!-- ====================================================
//		 analogInput type options
//	 	====================================================
//	 	enum ANALOG_SENSOR_TYPE
//	 	{
//	 		UNKNOWN_ANALOG_TYPE = -1,
//	 		ANALOG_GENERAL,
//	 		ANALOG_GYRO,
//	 		POTENTIOMETER,
//	 		PRESSURE_GAUGE,
//	 		MAX_ANALOG_TYPES
//	 	};
//	 	==================================================== -->
//	<!ELEMENT analogInput EMPTY>
//	<!ATTLIST analogInput
//  	      type              (  0 |  1 |  2 ) "0"
//      	  analogId          (  0 |  1 |  2 |  3 |  4 |  5 |  6 |  7 ) "0"
//            voltageMin        CDATA "0.0"
//            voltageMax        CDATA "5.0"
//            outputMin         CDATA #REQUIRED
//            outputMax         CDATA #REQUIRED
//	>
//
// --------------------------------------------------------------------------------------------

#pragma once


// C++ includes

// FRC includes
#include <frc/AnalogInput.h>

// Team302 includes
#include <hw/DragonAnalogInput.h>

// Third Party includes
#include <pugixml/pugixml.hpp>

using namespace frc;

class AnalogInputDefn
{
    public:

		AnalogInputDefn() = default;
        virtual ~AnalogInputDefn() = default;


        //-----------------------------------------------------------------------
        // Method:      ParseXML
        // Description: Parse a analogSensor XML element and create a DragonAnalogInput from
        //              its definition.
        // Returns:     DragonAnalogInput*      AnalogInput (or nullptr if XML
        //                                  	is ill-formed
        //-----------------------------------------------------------------------
        static DragonAnalogInput* ParseXML
        (
            pugi::xml_node      sensorNode
        );
};
