
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
#include <subsys/SwerveChassis.h>
#include <subsys/SwerveHelpers/WPIDirtySwerve.h>

// Third Party Includes

using namespace std;
using namespace frc;

WPIDirtySwerve::WPIDirtySwerve
(
    units::length::meter_t                  wheelBase,
    units::length::meter_t                  wheelTrack,
    units::velocity::meters_per_second_t    maxSpeed
) : m_wheelBase(wheelBase),
    m_wheelTrack(wheelTrack),
    m_maxSpeed(maxSpeed),
    m_frontLeftLocation(wheelBase/2.0, wheelTrack/2.0),
    m_frontRightLocation(wheelBase/2.0, -1.0*wheelTrack/2.0),
    m_backLeftLocation(-1.0*wheelBase/2.0, wheelTrack/2.0),
    m_backRightLocation(-1.0*wheelBase/2.0, -1.0*wheelTrack/2.0)
{

}

wpi::array<frc::SwerveModuleState, 4> WPIDirtySwerve::CalcModuleStates
(
    ChassisSpeeds                           speeds
) 
{
    auto states = m_kinematics.ToSwerveModuleStates(speeds);
    m_kinematics.DesaturateWheelSpeeds(&states, m_maxSpeed);
    return states;  
}

