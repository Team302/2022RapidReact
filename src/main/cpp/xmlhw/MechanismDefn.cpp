
//====================================================================================================================================================
/// Copyright 2022 Lake Orion Robotics FIRST Team 302
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

//========================================================================================================
/// @class MechansimDefn
/// @brief Create a mechaism from an XML definition 
//========================================================================================================

// C++ Includes
#include <memory>
#include <string>
#include <utility>

// FRC includes

// Team 302 includes
#include <hw/interfaces/IDragonMotorController.h>
#include <hw/usages/AnalogInputMap.h>
#include <hw/usages/DigitalInputMap.h>
#include <hw/usages/DragonSolenoidMap.h>
#include <hw/usages/IDragonMotorControllerMap.h>
#include <hw/usages/ServoMap.h>
#include <hw/DragonAnalogInput.h>
#include <subsys/interfaces/IMech.h>
#include <subsys/MechanismFactory.h>
#include <subsys/MechanismTypes.h>
#include <utils/Logger.h>
#include <xmlhw/AnalogInputDefn.h>
#include <xmlhw/CanCoderDefn.h>
#include <xmlhw/DigitalInputDefn.h>
#include <xmlhw/MechanismDefn.h>
#include <xmlhw/MotorDefn.h>
#include <xmlhw/ServoDefn.h> 
#include <xmlhw/SolenoidDefn.h>

// Third Party Includes
#include <pugixml/pugixml.hpp>


using namespace frc;
using namespace pugi;
using namespace std;



/// @brief  Parse a Mechanism XML element and create an IMechanism from its definition.
/// @return IMechanism*   pointer to the mechanism
void MechanismDefn::ParseXML
(
    xml_node      mechanismNode
)
{
    // initialize attributes
    MechanismTypes::MECHANISM_TYPE type = MechanismTypes::UNKNOWN_MECHANISM;

    bool hasError       = false;
    string networkTableName;
    string controlFileName;

    // Parse/validate xml
    for (xml_attribute attr = mechanismNode.first_attribute(); attr; attr = attr.next_attribute())
    {
        string attrName (attr.name());
        if ( attrName.compare("type") == 0 )
        {
            string typeStr = attr.as_string();
            for_each( typeStr.begin(), typeStr.end(), [](char & c){c = ::toupper(c);});

            if ( typeStr.compare( "LEFT_INTAKE") == 0 )
            {
                type = MechanismTypes::MECHANISM_TYPE::LEFT_INTAKE;
            }
            else if ( typeStr.compare( "RIGHT_INTAKE") == 0 )
            {
                type = MechanismTypes::MECHANISM_TYPE::RIGHT_INTAKE;
            }
            else if (typeStr.compare( "BALL_TRANSFER") == 0)
            {
                type = MechanismTypes::MECHANISM_TYPE::BALL_TRANSFER;
            }
            else if (typeStr.compare( "SHOOTER") == 0)
            {
                type = MechanismTypes::MECHANISM_TYPE::SHOOTER;
            }
            else if (typeStr.compare( "CLIMBER") == 0)
            {
                type = MechanismTypes::MECHANISM_TYPE::CLIMBER;
            }
            else if (typeStr.compare( "LIFT") == 0)
            {
                type = MechanismTypes::MECHANISM_TYPE::LIFT;
            }
            else if (typeStr.compare( "INDEXER") == 0)
            {
                type = MechanismTypes::MECHANISM_TYPE::INDEXER;
            }
            else
            {
                    string msg = "unknown Mechanism type ";
                    msg += attr.value();
                    Logger::GetLogger()->LogError( "MechanismDefn::ParseXML", msg );
                    hasError = true;
            }
        }
        else if ( attrName.compare("networkTable") == 0 )
        {
            networkTableName = attr.as_string();
        }
        else if ( attrName.compare("controlFile") == 0 )
        {
            controlFileName = attr.as_string();
        }
        else
        {
            string msg = "invalid attribute ";
            msg += attr.name();
            Logger::GetLogger()->LogError( "MechanismDefn::ParseXML", msg );
            hasError = true;
        }
    }

    // Parse/validate subobject xml
    unique_ptr<MotorDefn> motorXML = make_unique<MotorDefn>();
    unique_ptr<AnalogInputDefn> analogXML = make_unique<AnalogInputDefn>();
    unique_ptr<DigitalInputDefn> digitalXML = make_unique<DigitalInputDefn>();
    unique_ptr<ServoDefn> servoXML = make_unique<ServoDefn>();
    unique_ptr<SolenoidDefn> solenoidXML = make_unique<SolenoidDefn>();
    unique_ptr<CanCoderDefn> cancoderXML = make_unique<CanCoderDefn>();

    IDragonMotorControllerMap motors;
    ServoMap servos;
    DragonSolenoidMap solenoids;
    AnalogInputMap analogInputs;
    DigitalInputMap digitalInputs;
    shared_ptr<ctre::phoenix::sensors::CANCoder> canCoder = nullptr;

    for (xml_node child = mechanismNode.first_child(); child  && !hasError; child = child.next_sibling())
    {
        if ( strcmp( child.name(), "motor") == 0 )
        {
            auto motor = motorXML.get()->ParseXML(child);
            if ( motor.get() != nullptr )
            {
                motors[ motor.get()->GetType() ] =  motor ;
            }
        }
        else if ( strcmp( child.name(), "analogInput") == 0 )
        {
            auto analogIn = analogXML->ParseXML(child);
            if ( analogIn != nullptr )
            {
                analogInputs[analogIn->GetType()] = analogIn;
            }
        }
        else if ( strcmp( child.name(), "digitalInput") == 0 )
        {
            auto digitalIn = digitalXML->ParseXML(child);
            if ( digitalIn.get() != nullptr )
            {
                digitalInputs[digitalIn.get()->GetType()] = digitalIn;
            }
        }
        else if ( strcmp( child.name(), "servo") == 0 )
        {
            auto servo = servoXML->ParseXML(child);
            if ( servo != nullptr )
            {
                servos[servo->GetUsage()] = servo;
            }
        }
        else if ( strcmp( child.name(), "solenoid" ) == 0 )
        {
            auto sol = solenoidXML->ParseXML(child);
            if ( sol.get() != nullptr )
            {
                solenoids[sol.get()->GetType()] = sol;
            }
        }
        else if ( strcmp( child.name(), "canCoder" ) == 0)
        {
            canCoder = cancoderXML.get()->ParseXML(child);
        }
        else
        {
            string msg = "unknown child ";
            msg += child.name();
            Logger::GetLogger()->LogError( string("MechanismDefn"), msg );
        }
    }


    // create instance
    if ( !hasError )
    {
        MechanismFactory* factory =  MechanismFactory::GetMechanismFactory();
        factory->CreateIMechanism( type, 
                                   networkTableName,
                                   controlFileName,
                                   motors, 
                                   solenoids, 
                                   servos, 
                                   digitalInputs,
                                   analogInputs, 
                                   canCoder );
    }

}
