
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
#include <units/length.h>
#include <units/velocity.h>
#include <wpi/numbers>
#include <frc/kinematics/SwerveModuleState.h>

// Team 302 includes
#include <hw/DragonPigeon.h>
#include <subsys/SwerveChassis.h>
#include <subsys/SwerveHelpers/EtherDirtySwerve.h>
#include <utils/Logger.h>

// Third Party Includes

using namespace std;
using namespace frc;

EtherDirtySwerve::EtherDirtySwerve
(
    units::length::meter_t                  wheelBase,
    units::length::meter_t                  wheelTrack,
    units::velocity::meters_per_second_t    maxSpeed
) : m_wheelBase(wheelBase),
    m_wheelTrack(wheelTrack),
    m_maxSpeed(maxSpeed)
{

}

wpi::array<frc::SwerveModuleState, 4> EtherDirtySwerve::CalcModuleStates
(
    ChassisSpeeds                           speeds
) 
{
    // These calculations are based on Ether's Chief Delphi derivation
    // The only changes are that that derivation is based on positive angles being clockwise
    // and our codes/sensors are based on positive angles being counter clockwise.

    // A = Vx - omega * L/2
    // B = Vx + omega * L/2
    // C = Vy - omega * W/2
    // D = Vy + omega * W/2
    //
    // Where:
    // Vx is the sideways (strafe) vector
    // Vy is the forward vector
    // omega is the rotation about Z vector
    // L is the wheelbase (front to back)
    // W is the wheeltrack (side to side)
    //
    // Since our Vx is forward and Vy is strafe we need to rotate the vectors
    // We will use these variable names in the code to help tie back to the document.
    // Variable names, though, will follow C++ standards and start with a lower case letter.

    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Drive", speeds.vx.to<double>());
    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Strafe", speeds.vy.to<double>());
    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Rotate", speeds.omega.to<double>());

    auto l = m_wheelBase;
    auto w = m_wheelTrack;

    auto vy = 1.0 * speeds.vx;
    auto vx = -1.0 * speeds.vy;
    auto omega = speeds.omega;

    units::velocity::meters_per_second_t omegaL = omega.to<double>() * l / 2.0 / 1_s ;
    units::velocity::meters_per_second_t omegaW = omega.to<double>() * w / 2.0 / 1_s;
    
    auto a = vx - omegaL;
    auto b = vx + omegaL;
    auto c = vy - omegaW;
    auto d = vy + omegaW;

    // here we'll negate the angle to conform to the positive CCW convention
    SwerveModuleState flState;
    flState.angle = units::angle::radian_t(atan2(b.to<double>(), d.to<double>()));
    flState.angle = -1.0 * flState.angle.Degrees();
    flState.speed = units::velocity::meters_per_second_t(sqrt( pow(b.to<double>(),2) + pow(d.to<double>(),2) ));
    auto maxCalcSpeed = abs(flState.speed.to<double>());

    SwerveModuleState frState;
    frState.angle = units::angle::radian_t(atan2(b.to<double>(), c.to<double>()));
    frState.angle = -1.0 * frState.angle.Degrees();
    frState.speed = units::velocity::meters_per_second_t(sqrt( pow(b.to<double>(),2) + pow(c.to<double>(),2) ));
    if (abs(frState.speed.to<double>())>maxCalcSpeed)
    {
        maxCalcSpeed = abs(frState.speed.to<double>());
    }

    SwerveModuleState blState;
    blState.angle = units::angle::radian_t(atan2(a.to<double>(), d.to<double>()));
    blState.angle = -1.0 * blState.angle.Degrees();
    blState.speed = units::velocity::meters_per_second_t(sqrt( pow(a.to<double>(),2) + pow(d.to<double>(),2) ));
    if (abs(blState.speed.to<double>())>maxCalcSpeed)
    {
        maxCalcSpeed = abs(blState.speed.to<double>());
    }

    SwerveModuleState brState;
    brState.angle = units::angle::radian_t(atan2(a.to<double>(), c.to<double>()));
    brState.angle = -1.0 * brState.angle.Degrees();
    brState.speed = units::velocity::meters_per_second_t(sqrt( pow(a.to<double>(),2) + pow(c.to<double>(),2) ));
    if (abs(brState.speed.to<double>())>maxCalcSpeed)
    {
        maxCalcSpeed = abs(brState.speed.to<double>());
    }

    // normalize speeds if necessary (maxCalcSpeed > max attainable speed)
    if ( maxCalcSpeed > m_maxSpeed.to<double>() )
    {
        auto ratio = m_maxSpeed.to<double>() / maxCalcSpeed;
        flState.speed *= ratio;
        frState.speed *= ratio;
        blState.speed *= ratio;
        brState.speed *= ratio;
    }

    wpi::array<SwerveModuleState, 4> states = {flState, frState, blState, brState};
    return states;
}
