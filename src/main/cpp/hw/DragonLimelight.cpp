
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
#include <string>
#include <vector>
#include <cmath>

// FRC includes
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>

// Team 302 includes
#include <hw/DragonLimelight.h>
#include <utils/Logger.h>

// Third Party Includes

using namespace nt;
using namespace std;

///-----------------------------------------------------------------------------------
/// Method:         DragonLimelight (constructor)
/// Description:    Create the object
///-----------------------------------------------------------------------------------
DragonLimelight::DragonLimelight
(
    string                      tableName,                  /// <I> - network table name
    units::length::inch_t       mountingHeight,             /// <I> - mounting height of the limelight
    units::length::inch_t       mountingHorizontalOffset,   /// <I> - mounting horizontal offset from the middle of the robot,
    units::angle::degree_t      rotation,                   /// <I> - clockwise rotation of limelight
    units::angle::degree_t      mountingAngle,              /// <I> - mounting angle of the camera
    units::length::inch_t       targetHeight,               /// <I> - height the target
    units::length::inch_t       targetHeight2               /// <I> - height of second target
) : //IDragonSensor(),
    //IDragonDistanceSensor(),
    m_networktable( NetworkTableInstance::GetDefault().GetTable( tableName.c_str()) ),
    m_mountHeight( mountingHeight ),
    m_mountingHorizontalOffset( mountingHorizontalOffset ),
    m_rotation(rotation),
    m_mountingAngle( mountingAngle ),
    m_targetHeight( targetHeight ),
    m_targetHeight2( targetHeight2 )
{
    //SetLEDMode( DragonLimelight::LED_MODE::LED_OFF);
}

std::vector<double> DragonLimelight::Get3DSolve() const
{
    std::vector<double> output;
    return output;
    //return m_networktable.get()->GetNumberArray("camtran", 0);
}

bool DragonLimelight::HasTarget() const
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return ( nt->GetNumber("tv", 0.0) > 0.1 );
    }
    return false;
}

units::angle::degree_t DragonLimelight::GetTx() const
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return units::angle::degree_t(nt->GetNumber("tx", 0.0));
    }
    return units::angle::degree_t(0.0);
}
 
units::angle::degree_t DragonLimelight::GetTy() const
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return units::angle::degree_t(nt->GetNumber("ty", 0.0));
    }
    return units::angle::degree_t(0.0);
}

units::angle::degree_t DragonLimelight::GetTargetHorizontalOffset() const
{
    if ( abs(m_rotation.to<double>()) < 1.0 )
    {
        return GetTx();
    }
    else if ( abs(m_rotation.to<double>()-90.0) < 1.0 )
    {
        return -1.0 * GetTy();
    }
    else if ( abs(m_rotation.to<double>()-180.0) < 1.0 )
    {
        return -1.0 * GetTx();
    }
    else if ( abs(m_rotation.to<double>()-270.0) < 1.0 )
    {
        return GetTy();
    }
    Logger::GetLogger()->LogError("DragonLimelight::GetTargetVerticalOffset", "Invalid limelight rotation");
    return GetTx();
}

units::angle::degree_t DragonLimelight::GetTargetVerticalOffset() const
{
    if ( abs(m_rotation.to<double>()) < 1.0 )
    {
        return GetTy();
    }
    else if ( abs(m_rotation.to<double>()-90.0) < 1.0 )
    {
        return GetTx();
    }
    else if ( abs(m_rotation.to<double>()-180.0) < 1.0 )
    {
        return -1.0 * GetTy();
    }
    else if ( abs(m_rotation.to<double>()-270.0) < 1.0 )
    {
        return -1.0 * GetTx();
    }
    Logger::GetLogger()->LogError("DragonLimelight::GetTargetVerticalOffset", "Invalid limelight rotation");
    return GetTy();   
}

double DragonLimelight::GetTargetArea() const
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return nt->GetNumber("ta", 0.0);
    }
    return 0.0;
}

units::angle::degree_t DragonLimelight::GetTargetSkew() const
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return units::angle::degree_t(nt->GetNumber("ts", 0.0));
    }
    return units::angle::degree_t(0.0);
}

units::time::microsecond_t DragonLimelight::GetPipelineLatency() const
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return units::time::second_t(nt->GetNumber("tl", 0.0));
    }
    return units::time::second_t(0.0);
}


void DragonLimelight::SetTargetHeight
(
    units::length::inch_t targetHeight
)
{
    m_targetHeight = targetHeight;
}

void DragonLimelight::SetLEDMode(DragonLimelight::LED_MODE mode)
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        nt->PutNumber("ledMode", mode);
    }
}

void DragonLimelight::SetCamMode(DragonLimelight::CAM_MODE mode)
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        nt->PutNumber("camMode", mode);
    }
}

void DragonLimelight::SetPipeline(int pipeline)
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        nt->PutNumber("pipeline", pipeline);
    }
}

void DragonLimelight::SetStreamMode(DragonLimelight::STREAM_MODE mode)
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        nt->PutNumber("stream", mode);
    }
}

void DragonLimelight::SetCrosshairPos( double crosshairPosX, double crosshairPosY)
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        nt->PutNumber("cx0", crosshairPosX);
        nt->PutNumber("cy0", crosshairPosY);
    }
}

void DragonLimelight::SetSecondaryCrosshairPos( double crosshairPosX, double crosshairPosY)
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        nt->PutNumber("cx1", crosshairPosX);
        nt->PutNumber("cy1", crosshairPosY);
    }
}

// MAX of 32 snapshots can be saved
void DragonLimelight::ToggleSnapshot(DragonLimelight::SNAPSHOT_MODE toggle)
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        nt->PutNumber("snapshot", toggle);
    }
}

void DragonLimelight::PrintValues()
{
    Logger::GetLogger()->LogError( "DragonLimelight::PrintValues HasTarget", to_string( HasTarget() ) );    
    Logger::GetLogger()->LogError( "DragonLimelight::PrintValues XOffset", to_string( GetTargetHorizontalOffset().to<double>() ) ); 
    Logger::GetLogger()->LogError( "DragonLimelight::PrintValues YOffset", to_string( GetTargetVerticalOffset().to<double>() ) ); 
    Logger::GetLogger()->LogError( "DragonLimelight::PrintValues Area", to_string( GetTargetArea() ) ); 
    Logger::GetLogger()->LogError( "DragonLimelight::PrintValues Skew", to_string( GetTargetSkew().to<double>() ) ); 
    Logger::GetLogger()->LogError( "DragonLimelight::PrintValues Latency", to_string( GetPipelineLatency().to<double>() ) ); 
}

units::length::inch_t DragonLimelight::EstimateTargetDistance() const
{
    units::angle::degree_t angleFromHorizon = (GetMountingAngle() + GetTargetVerticalOffset());
    units::angle::radian_t angleRad = angleFromHorizon;
    double tanAngle = tan(angleRad.to<double>());

    auto deltaHgt = GetTargetHeight()-GetMountingHeight();

    Logger::GetLogger()->ToNtTable(m_networktable, string("mounting angle "), GetMountingAngle().to<double>());
    Logger::GetLogger()->ToNtTable(m_networktable, string("target vertical angle "), GetTargetVerticalOffset().to<double>());
    Logger::GetLogger()->ToNtTable(m_networktable, string("angle radians "), angleRad.to<double>());
    Logger::GetLogger()->ToNtTable(m_networktable, string("deltaH "), deltaHgt.to<double>());
    Logger::GetLogger()->ToNtTable(m_networktable, string("tan angle "), tanAngle);
    Logger::GetLogger()->ToNtTable(m_networktable, string("distance "), ((GetTargetHeight()-GetMountingHeight()) / tanAngle).to<double>());

    return (GetTargetHeight()-GetMountingHeight()) / tanAngle;
}


