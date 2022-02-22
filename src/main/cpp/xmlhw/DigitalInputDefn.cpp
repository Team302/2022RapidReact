
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

// C++ includes
#include <string>
#include <memory>

// FRC includes

// Team302 includes
#include <hw/DragonDigitalInput.h>
#include <hw/usages/DigitalInputUsage.h>
#include <utils/HardwareIDValidation.h>
#include <utils/Logger.h>
#include <xmlhw/DigitalInputDefn.h>

// Third Party includes
#include <pugixml/pugixml.hpp>


using namespace frc;
using namespace std;
using namespace pugi;

//-----------------------------------------------------------------------
// Method:      ParseXML
// Description: Parse a motor XML element and create a DragonTalon from
//              its definition.
//
// Returns:     DragonDigitalInput*
//-----------------------------------------------------------------------
shared_ptr<DragonDigitalInput> DigitalInputDefn::ParseXML
(
    xml_node DigitalNode
)
{
    // initialize the output
    shared_ptr<DragonDigitalInput> input;

    // initialize the attributes to default values
    auto usage = DigitalInputUsage::DIGITAL_SENSOR_USAGE::UNKNOWN_DIGITAL_TYPE;
    int  digitalID = 0;
    bool reversed = false;
    bool hasError = false;

    // Parse/validate the XML
    for (pugi::xml_attribute attr = DigitalNode.first_attribute(); attr; attr = attr.next_attribute())
    {
        string attrName (attr.name());
        if (attrName.compare("usage") == 0)
        {
            auto usageString = string(attr.value());
            usage = DigitalInputUsage::GetInstance()->GetUsage(usageString );
        }
        else if (attrName.compare("digitalId") == 0)
        {
            digitalID = attr.as_int();
            hasError = HardwareIDValidation::ValidateDIOID( digitalID, string( "DigitalInputDefn::ParseXML(digital Input pin)" ) );
        }
        else if (attrName.compare("reversed" ) == 0)
        {
            reversed = attr.as_bool();

        }
        else
        {
            string msg = "unknown attribute ";
            msg += attr.name();
            Logger::GetLogger()->LogError( string("DigitalInputDefn "), msg );
        }
    }

    // Create the DragonDigitalInput
    if ( !hasError )
    {
        input = make_shared<DragonDigitalInput>( usage, digitalID, reversed );
    }
    return input;
}

