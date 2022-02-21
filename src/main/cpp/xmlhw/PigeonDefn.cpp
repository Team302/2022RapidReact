
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
#include <xmlhw/PigeonDefn.h>
#include <hw/DragonPigeon.h>
#include <utils/HardwareIDValidation.h>
#include <utils/Logger.h>
#include <hw/factories/PigeonFactory.h>

// Third Party Includes
#include <pugixml/pugixml.hpp>

using namespace std;
using namespace pugi;

//-----------------------------------------------------------------------
// Method:      ParseXML
// Description: Parse a pigeon XML element and create a DragonPigeon from
//              its definition.
// Returns:     DragonPigeon*       pigeon IMU (or nullptr if XML is ill-formed)
//-----------------------------------------------------------------------
DragonPigeon* PigeonDefn::ParseXML
(
    xml_node      pigeonNode
)
{
    // initialize output
    DragonPigeon* pigeon = nullptr;

    // initialize attributes to default values
    int canID = 0;
    double rotation = 0.0;
    DragonPigeon::PIGEON_TYPE type = DragonPigeon::PIGEON_TYPE::PIGEON1;
    DragonPigeon::PIGEON_USAGE usage = DragonPigeon::PIGEON_USAGE::CENTER_OF_ROBOT;

    bool hasError = false;

    // parse/validate xml
    for (xml_attribute attr = pigeonNode.first_attribute(); attr; attr = attr.next_attribute())
    {
        if ( strcmp( attr.name(), "canId" ) == 0 )
        {
            canID = attr.as_int();
            hasError = HardwareIDValidation::ValidateCANID( canID, string( "Pigeon::ParseXML" ) );
        }
        else if ( strcmp( attr.name(), "rotation") == 0 )
        {
            rotation = attr.as_double();
        }
        else if (strcmp(attr.name(), "type") == 0)
        {
            if (strcmp(attr.value(), "PIGEON2") == 0)
            {
                type = DragonPigeon::PIGEON_TYPE::PIGEON2;
            }
            else 
            {
                type = DragonPigeon::PIGEON_TYPE::PIGEON1;
            }
        }
        else if (strcmp(attr.name(), "usage") == 0)
        {
            if (strcmp(attr.value(), "CENTER_OF_SHOOTER") == 0)
            {
                usage = DragonPigeon::PIGEON_USAGE::CENTER_OF_SHOOTER;
            }
            else 
            {
                usage = DragonPigeon::PIGEON_USAGE::CENTER_OF_ROBOT;
            }
        }
        else
        {
            string msg("Invalid attribute ");
            msg += attr.name();
            Logger::GetLogger()->LogError( Logger::LOGGER_LEVEL::ERROR, string("PigeonDefn::ParseXML"), msg );
            hasError = true;
        }

    }

    if ( !hasError )
    {
        Logger::GetLogger()->OnDash(string("RobotXML Parsing"), string("Create Pigeon"));
        pigeon = PigeonFactory::GetFactory()->CreatePigeon( canID, 
                                                            type,
                                                            usage,
                                                            rotation );
    }
    return pigeon;
}