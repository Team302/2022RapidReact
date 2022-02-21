
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

// FRC includes

// Team 302 includes
#include <hw/DragonServo.h>
#include <hw/factories/DragonServoFactory.h>
#include <hw/usages/ServoUsage.h>
#include <utils/HardwareIDValidation.h>
#include <utils/Logger.h>
#include <xmlhw/ServoDefn.h>

// Third Party Includes
#include <pugixml/pugixml.hpp>

using namespace std;

    //-----------------------------------------------------------------------
    // Method:      ParseXML
    // Description: Parse a servo XML element and create a DragonServo from
    //              its definition.
    //
    //
    // Returns:     void        
    //-----------------------------------------------------------------------
    DragonServo* ServoDefn::ParseXML
    (
        pugi::xml_node      ServoNode
    )
    {
        DragonServo* servo = nullptr; 

        // initialize attributes to default values
        int pwmID = 0;
        ServoUsage::SERVO_USAGE usage = ServoUsage::UNKNOWN_SERVO_USAGE;
        double minAngle = 0.0;
        double maxAngle = 360.0;

        bool hasError = false;

        // parse/validate the xml
        for (pugi::xml_attribute attr = ServoNode.first_attribute(); attr && !hasError; attr = attr.next_attribute())
        {
            string attrName (attr.name());
            if (attrName.compare("usage") == 0)
            {
                usage = ServoUsage::GetInstance()->GetUsage( string( attr.value()));
            }
            else if (attrName.compare("pwmId" ) == 0)
            {
                pwmID = attr.as_int();
                hasError = HardwareIDValidation::ValidateDIOID( pwmID, string( "ServoDefn::ParseXML(PWM ID)" ) );
            }
            else if (attrName.compare("minAngle" ) == 0)
            {
                minAngle = attr.as_int();
            }
            else if (attrName.compare("maxAngle" ) == 0)
            {
                maxAngle = attr.as_int();
            }
            else
            {
                string msg = "unknown attribute ";
                msg += attr.name();
                Logger::GetLogger()->LogError( string("ServoDefn"), msg );
                hasError = true;
            }
        }

        // create the object
        if ( !hasError )
        {
            servo = DragonServoFactory::GetInstance()->CreateDragonServo( usage, pwmID, minAngle, maxAngle);
        }
        return servo;
    }
