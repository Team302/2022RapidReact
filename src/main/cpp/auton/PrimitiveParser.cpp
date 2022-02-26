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

#include <map>
#include <iostream>

#include <frc/Filesystem.h>

#include <auton/AutonSelector.h>
#include <auton/PrimitiveEnums.h>
#include <auton/PrimitiveParams.h>
#include <auton/PrimitiveParser.h>
#include <auton/primitives/IPrimitive.h>
#include <utils/Logger.h>

#include <pugixml/pugixml.hpp>


using namespace std;
using namespace pugi;

PrimitiveParamsVector PrimitiveParser::ParseXML
(
    string     fileName
)
{

    PrimitiveParamsVector paramVector;

    auto primitiveType = UNKNOWN_PRIMITIVE;
    auto time = 15.0;
    auto distance = 0.0;
    auto headingOption = IChassis::HEADING_OPTION::MAINTAIN;
    auto heading = 0.0;
    auto startDriveSpeed = 0.0;
    auto endDriveSpeed = 0.0;
    auto xloc = 0.0;
    auto yloc = 0.0;
    std::string pathName;
    auto leftIntakeState = IntakeStateMgr::INTAKE_STATE::OFF;
    auto rightIntakeState = IntakeStateMgr::INTAKE_STATE::OFF;
    auto shooterState = ShooterStateMgr::SHOOTER_STATE::PREPARE_TO_SHOOT;

    auto hasError = false;

	auto deployDir = frc::filesystem::GetDeployDirectory();
	auto autonDir = deployDir + "/auton/";

    string fulldirfile = autonDir;
    fulldirfile += fileName;
    // initialize the xml string to enum maps
    map<string, PRIMITIVE_IDENTIFIER> primStringToEnumMap;
    primStringToEnumMap["DO_NOTHING"] = DO_NOTHING;
    primStringToEnumMap["HOLD_POSITION"]  = HOLD_POSITION;
    primStringToEnumMap["DRIVE_DISTANCE"] = DRIVE_DISTANCE;
    primStringToEnumMap["DRIVE_TIME"] = DRIVE_TIME;
    primStringToEnumMap["DRIVE_TO_WALL"] = DRIVE_TO_WALL;
    primStringToEnumMap["TURN_ANGLE_ABS"] = TURN_ANGLE_ABS;
    primStringToEnumMap["TURN_ANGLE_REL"] = TURN_ANGLE_REL;
    primStringToEnumMap["DRIVE_PATH"] = DRIVE_PATH;
    primStringToEnumMap["RESET_POSITION"] = RESET_POSITION;

    map<string, IChassis::HEADING_OPTION> headingOptionMap;
    headingOptionMap["MAINTAIN"] = IChassis::HEADING_OPTION::MAINTAIN;
    headingOptionMap["TOWARD_GOAL"] = IChassis::HEADING_OPTION::TOWARD_GOAL;
    headingOptionMap["LEFT_INTAKE_TOWARD_BALL"] = IChassis::HEADING_OPTION::LEFT_INTAKE_TOWARD_BALL;
    headingOptionMap["RIGHT_INTAKE_TOWARD_BALL"] = IChassis::HEADING_OPTION::RIGHT_INTAKE_TOWARD_BALL;
    headingOptionMap["SPECIFIED_ANGLE"] = IChassis::HEADING_OPTION::SPECIFIED_ANGLE;
    
    map<string, IntakeStateMgr::INTAKE_STATE> intakeStateMap;
    intakeStateMap["OFF"] = IntakeStateMgr::INTAKE_STATE::OFF;
    intakeStateMap["INTAKE"] = IntakeStateMgr::INTAKE_STATE::INTAKE;
    intakeStateMap["EXPEL"] = IntakeStateMgr::INTAKE_STATE::EXPEL;  

    map<string, ShooterStateMgr::SHOOTER_STATE> shooterStateMap;
    shooterStateMap["OFF"] = ShooterStateMgr::SHOOTER_STATE::OFF;
    shooterStateMap["AUTO_SHOOT_HIGH_GOAL_FAR"] = ShooterStateMgr::SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_FAR;
    shooterStateMap["AUTO_SHOOT_HIGH_GOAL_CLOSE"] = ShooterStateMgr::SHOOTER_STATE::AUTO_SHOOT_HIGH_GOAL_CLOSE;
    shooterStateMap["SHOOT_LOW_GOAL"] = ShooterStateMgr::SHOOTER_STATE::SHOOT_LOW_GOAL;
    shooterStateMap["PREPARE_TO_SHOOT"] = ShooterStateMgr::SHOOTER_STATE::PREPARE_TO_SHOOT;
    shooterStateMap["MANUAL_SHOOT"] = ShooterStateMgr::SHOOTER_STATE::SHOOT_MANUAL;

    xml_document doc;
    xml_parse_result result = doc.load_file( fulldirfile.c_str() );
   
    if ( result )
    {
        xml_node auton = doc.root();
        for (xml_node node = auton.first_child(); node; node = node.next_sibling())
        {
            for (xml_node primitiveNode = node.first_child(); primitiveNode; primitiveNode = primitiveNode.next_sibling())
            {
                if ( strcmp( primitiveNode.name(), "primitive") == 0 )
                {

                    for (xml_attribute attr = primitiveNode.first_attribute(); attr; attr = attr.next_attribute())
                    {
                        std::cout << "PrimParser atribute name: " << attr.name() << std::endl;
                        if ( strcmp( attr.name(), "id" ) == 0 )
                        {
                            auto paramStringToEnumItr = primStringToEnumMap.find( attr.value() );
                            if ( paramStringToEnumItr != primStringToEnumMap.end() )
                            {
                                primitiveType = paramStringToEnumItr->second;
                            }
                            else
                            {
                                Logger::GetLogger()->LogError( string("PrimitiveParser::ParseXML invalid id"), attr.value());
                                hasError = true;
                            }
                        }
                        else if ( strcmp( attr.name(), "time" ) == 0 )
                        {
                            time = attr.as_float();
                        }
                        else if ( strcmp( attr.name(), "distance" ) == 0 )
                        {
                            distance = attr.as_float();
                        }
                        else if ( strcmp( attr.name(), "headingOption" ) == 0 )
                        {
                            auto headingItr = headingOptionMap.find( attr.value() );
                            if ( headingItr != headingOptionMap.end() )
                            {
                                headingOption = headingItr->second;
                            }
                            else
                            {
                                Logger::GetLogger()->LogError( string("PrimitiveParser::ParseXML invalid heading option"), attr.value());
                                hasError = true;
                            }
                        }
                        else if ( strcmp( attr.name(), "heading" ) == 0 )
                        {
                            heading = attr.as_float();
                        }
                        else if ( strcmp( attr.name(), "drivespeed" ) == 0 )
                        {
                            startDriveSpeed = attr.as_float();
                        }
                        else if ( strcmp( attr.name(), "enddrivespeed" ) == 0 )
                        {
                            endDriveSpeed = attr.as_float();
                        }
                        else if ( strcmp( attr.name(), "xloc" ) == 0 )
                        {
                            xloc = attr.as_float();
                        }
                        else if ( strcmp( attr.name(), "yloc" ) == 0 )
                        {
                            yloc = attr.as_float();
                        }
                        else if ( strcmp( attr.name(), "pathname") == 0)
                        {
                            pathName = attr.value();
                        }                
                        else if ( strcmp( attr.name(), "leftIntake" ) == 0 )
                        {
                            auto leftItr = intakeStateMap.find( attr.value() );
                            if ( leftItr != intakeStateMap.end() )
                            {
                                leftIntakeState = leftItr->second;
                            }
                            else
                            {
                                Logger::GetLogger()->LogError( string("PrimitiveParser::ParseXML invalid left intake state"), attr.value());
                                hasError = true;
                            }
                        }
                        else if ( strcmp( attr.name(), "rightIntake" ) == 0 )
                        {
                            auto rightItr = intakeStateMap.find( attr.value() );
                            if ( rightItr != intakeStateMap.end() )
                            {
                                rightIntakeState = rightItr->second;
                            }
                            else
                            {
                                Logger::GetLogger()->LogError( string("PrimitiveParser::ParseXML invalid right intake state"), attr.value());
                                hasError = true;
                            }
                        }
                        else if ( strcmp( attr.name(), "shooter" ) == 0 )
                        {
                            auto shootItr = shooterStateMap.find( attr.value() );
                            std::cout << "ShootItr second: " << to_string(shootItr->second) << endl;
                            std::cout << "ShootItr first: " << shootItr->first << endl;
                            if ( shootItr != shooterStateMap.end() )
                            {
                                shooterState = shootItr->second;
                                std::cout << "PrimParser State: " << to_string(shooterState) << std::endl;
                            }
                            else
                            {
                                Logger::GetLogger()->LogError( string("PrimitiveParser::ParseXML invalid shooter state"), attr.value());
                                hasError = true;
                            }
                        }
                        else
                        {
                            Logger::GetLogger()->LogError( string("PrimitiveParser::ParseXML invalid attribute"), attr.name());
                            hasError = true;
                        }
                    }
                    if ( !hasError )
                    {   
                        paramVector.emplace_back( new PrimitiveParams( primitiveType,
                                                                       time,
                                                                       distance,
                                                                       xloc,
                                                                       yloc,
                                                                       headingOption,
                                                                       heading,
                                                                       startDriveSpeed,
                                                                       endDriveSpeed,
                                                                       pathName,
                                                                       leftIntakeState,
                                                                       rightIntakeState,
                                                                       shooterState ) );
                    }
                    else 
                    {
                         Logger::GetLogger() -> LogError( string("PrimitiveParser::ParseXML"), string("Has Error"));
                    }
                }
            }
        }
    }
    else
    {
        Logger::GetLogger()->LogError( string("PrimitiveParser::ParseXML error parsing file"), fileName );
        Logger::GetLogger()->LogError( string("PrimitiveParser::ParseXML error message"), result.description() );
    }
    return paramVector;
}
