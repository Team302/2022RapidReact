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
#include <string>

#include <frc/Filesystem.h>

#include <auton/AutonSelector.h>
#include <auton/PrimitiveEnums.h>
#include <auton/PrimitiveParams.h>
#include <auton/PrimitiveParser.h>
#include <auton/primitives/IPrimitive.h>
#include <states/intake/LeftIntakeStateMgr.h>
#include <states/intake/RightIntakeStateMgr.h>
#include <states/shooter/ShooterStateMgr.h>
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
    headingOptionMap["TOWARD_GOAL_DRIVE"] = IChassis::HEADING_OPTION::TOWARD_GOAL_DRIVE;
    headingOptionMap["TOWARD_GOAL_LAUNCHPAD"] = IChassis::HEADING_OPTION::TOWARD_GOAL_LAUNCHPAD;
    headingOptionMap["LEFT_INTAKE_TOWARD_BALL"] = IChassis::HEADING_OPTION::LEFT_INTAKE_TOWARD_BALL;
    headingOptionMap["RIGHT_INTAKE_TOWARD_BALL"] = IChassis::HEADING_OPTION::RIGHT_INTAKE_TOWARD_BALL;
    headingOptionMap["SPECIFIED_ANGLE"] = IChassis::HEADING_OPTION::SPECIFIED_ANGLE;
    
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
                    
                    for (xml_attribute attr = primitiveNode.first_attribute(); attr; attr = attr.next_attribute())
                    {
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
                            auto leftItr = LeftIntakeStateMgr::GetInstance()->m_intakeXmlStringToStateEnumMap.find( attr.value() );
                            if ( leftItr != LeftIntakeStateMgr::GetInstance()->m_intakeXmlStringToStateEnumMap.end() )
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
                            auto rightItr = RightIntakeStateMgr::GetInstance()->m_intakeXmlStringToStateEnumMap.find( attr.value() );
                            if ( rightItr != RightIntakeStateMgr::GetInstance()->m_intakeXmlStringToStateEnumMap.end() )
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
                            auto shootItr = ShooterStateMgr::GetInstance()->m_shooterXmlStringToStateEnumMap.find( attr.value() );
                            if ( shootItr != ShooterStateMgr::GetInstance()->m_shooterXmlStringToStateEnumMap.end() )
                            {
                                shooterState = shootItr->second;
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
                        string ntName = string("Primitive ") + to_string(paramVector.size());
                        int slot = paramVector.size() - 1;
                        auto logger = Logger::GetLogger();
                        auto param = paramVector[slot];
                        logger->ToNtTable(ntName, string("Primitive ID"), to_string(param->GetID()));
                        logger->ToNtTable(ntName, string("Time"), param->GetTime());
                        logger->ToNtTable(ntName, string("Distance"), param->GetDistance());
                        logger->ToNtTable(ntName, string("X Location"), param->GetXLocation());
                        logger->ToNtTable(ntName, string("Y Location"), param->GetYLocation());
                        logger->ToNtTable(ntName, string("Heading Option"), to_string(param->GetHeadingOption()));
                        logger->ToNtTable(ntName, string("Heading"), param->GetHeading());
                        logger->ToNtTable(ntName, string("Drive Speed"), param->GetDriveSpeed());
                        logger->ToNtTable(ntName, string("End Drive Speed"), param->GetEndDriveSpeed());
                        logger->ToNtTable(ntName, string("Path Name"), param->GetPathName());
                        logger->ToNtTable(ntName, string("Left Intake"), to_string(param->GetLeftIntakeState()));
                        logger->ToNtTable(ntName, string("Right Intake"), to_string(param->GetRightIntakeState()));
                        logger->ToNtTable(ntName, string("Shooter"), to_string(param->GetShooterState()));
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
