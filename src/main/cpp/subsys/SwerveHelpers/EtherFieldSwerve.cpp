
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
#include <cmath>

// FRC includes
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <units/length.h>
#include <units/velocity.h>
#include <wpi/numbers>

// Team 302 includes
#include <hw/DragonPigeon.h>
#include <subsys/SwerveChassis.h>
#include <subsys/SwerveHelpers/EtherFieldSwerve.h>

// Third Party Includes

using namespace std;
using namespace frc;

EtherFieldSwerve::EtherFieldSwerve
(
    DragonPigeon*                           pigeon, 
    units::length::meter_t                  wheelBase,
    units::length::meter_t                  wheelTrack,
    units::velocity::meters_per_second_t    maxSpeed
) : EtherDirtySwerve(wheelBase, wheelTrack, maxSpeed),
    m_pigeon(pigeon)
{

}

wpi::array<frc::SwerveModuleState, 4> EtherFieldSwerve::CalcModuleStates
(
    ChassisSpeeds                           speeds
) 
{
    units::angle::radian_t yaw{m_pigeon->GetYaw()*wpi::numbers::pi/180.0};
    auto temp = speeds.vx*cos(yaw.to<double>()) + speeds.vy*sin(yaw.to<double>());
    auto strafe = -1.0*speeds.vx*sin(yaw.to<double>()) + speeds.vy*cos(yaw.to<double>());
    auto forward = temp;

    ChassisSpeeds newSpeeds{forward, strafe, speeds.omega};
    return EtherDirtySwerve::CalcModuleStates(newSpeeds);  
}

