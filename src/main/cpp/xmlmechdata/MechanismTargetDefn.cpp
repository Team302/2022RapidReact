
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
#include <array>
#include <string>
#include <cstring>

// FRC includes

// Team 302 includes
#include <controllers/MechanismTargetData.h>
#include <utils/Logger.h>
#include <xmlmechdata/MechanismTargetDefn.h>

// Third Party Includes
#include <pugixml/pugixml.hpp>

using namespace std;
using namespace pugi;


/// @brief      Parse MechanismTargetData XML element 
/// @param [in] pugi::xml_node  mechanism data node
/// @return     MechanismTargetData*       mechanism data
MechanismTargetData*  MechanismTargetDefn::ParseXML
(
    xml_node      MechanismDataNode
)
{
    // initialize output
    MechanismTargetData* mechData = nullptr;

    bool hasError = false;

    string stateName;
    string controllerIdentifier;
    string controllerIdentifier2;
    double target = 0.0;
    double secondTarget = 0.0;
    double robotPitch = 0.0;
    MechanismTargetData::SOLENOID solenoid = MechanismTargetData::SOLENOID::NONE; 
    array<double,3> function1Coeff = {0.0, 0.0, 0.0};
    array<double,3> function2Coeff = {0.0, 0.0, 0.0};


    // parse/validate xml
    for (xml_attribute attr = MechanismDataNode.first_attribute(); attr; attr = attr.next_attribute())
    {
        if ( strcmp( attr.name(), "stateIdentifier" ) == 0 )
        {
            stateName = string( attr.value() );
        }
        else if ( strcmp( attr.name(), "controlDataIdentifier" ) == 0 )
        {
            controllerIdentifier = string( attr.value() );
        }
        else if ( strcmp( attr.name(), "controlDataIdentifier2" ) == 0 )
        {
            controllerIdentifier2 = string( attr.value() );
        }
        else if ( strcmp( attr.name(), "value") == 0 )
        {
            target = attr.as_double();
        }
        else if ( strcmp( attr.name(), "secondValue") == 0 )
        {
            secondTarget = attr.as_double();
        }
        else if ( strcmp( attr.name(), "robotPitch") == 0 )
        {
            robotPitch = attr.as_double();
        }
        else if ( strcmp( attr.name(), "function1A") == 0 )
        {
            function1Coeff[0] = attr.as_double();
        }
        else if ( strcmp( attr.name(), "function1B") == 0 )
        {
            function1Coeff[1] = attr.as_double();
        }
        else if ( strcmp( attr.name(), "function1C") == 0 )
        {
            function1Coeff[2] = attr.as_double();
        }
        else if ( strcmp( attr.name(), "function2A") == 0 )
        {
            function2Coeff[0] = attr.as_double();
        }
        else if ( strcmp( attr.name(), "function2B") == 0 )
        {
            function2Coeff[1] = attr.as_double();
        }
        else if ( strcmp( attr.name(), "function2C") == 0 )
        {
            function2Coeff[2] = attr.as_double();
        }
        else if( strcmp( attr.name(), "solenoid" ) == 0 )
        {
            auto val = attr.value();
            if(strcmp(val, "ON") == 0)
            {
                solenoid = MechanismTargetData::SOLENOID::ON;
            }
            else if ( strcmp(val, "REVERSE") == 0)
            {
                solenoid = MechanismTargetData::SOLENOID::REVERSE;
            }
            else if( strcmp(val, "NONE") == 0)
            {
                solenoid = MechanismTargetData::SOLENOID::NONE;
            }
            else
            {
                Logger::GetLogger()->LogError( string("MechanismTargetDefn::ParseXML"), string("solenoid enum"));
            }
        }
        else
        {
            string msg = "unknown attribute ";
            msg += attr.name();
            Logger::GetLogger()->LogError( string("MechanismTargetDefn::ParseXML"), msg );
            hasError = true;
        }
    }

    if ( !hasError && !stateName.empty() && !controllerIdentifier.empty() )
    {
        mechData = new MechanismTargetData( stateName, 
                                            controllerIdentifier, 
                                            controllerIdentifier2, 
                                            target, 
                                            secondTarget,
                                            robotPitch,
                                            solenoid,
                                            function1Coeff,
                                            function2Coeff );
    }
    else
    {
        Logger::GetLogger()->LogError( string("MechanismTargetDefn::ParseXML"), string("incomplete date"));
    }

    return mechData;
}


