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

//C++ Includes
#include <memory>

// FRC includes

//Team302 Includes
#include <subsys/swerve/SwerveChassis.h>

#include <utils/Logger.h>

/// @brief Construct a swerve chassis
/// @param [in] std::shared_ptr<SwerveModule>           frontleft:          front left swerve module
/// @param [in] std::shared_ptr<SwerveModule>           frontright:         front right swerve module
/// @param [in] std::shared_ptr<SwerveModule>           backleft:           back left swerve module
/// @param [in] std::shared_ptr<SwerveModule>           backright:          back right swerve module
/// @param [in] units::length::inch_t                   wheelBase:          distance between the front and rear wheels
/// @param [in] units::length::inch_t                   track:              distance between the left and right wheels
/// @param [in] units::velocity::meters_per_second_t    maxSpeed:           maximum linear speed of the chassis 
/// @param [in] units::radians_per_second_t             maxAngularSpeed:    maximum rotation speed of the chassis 
/// @param [in] double                                  maxAcceleration:    maximum acceleration in meters_per_second_squared


SwerveChassis::SwerveChassis
(
    std::shared_ptr<SwerveModule>                                    frontLeft, 
    std::shared_ptr<SwerveModule>                                    frontRight,
    std::shared_ptr<SwerveModule>                                    backLeft, 
    std::shared_ptr<SwerveModule>                                    backRight, 
    units::length::inch_t                                       wheelDiameter,
    units::length::inch_t                                       wheelBase,
    units::length::inch_t                                       track,
    units::velocity::meters_per_second_t                        maxSpeed,
    units::radians_per_second_t                                 maxAngularSpeed,
    units::acceleration::meters_per_second_squared_t            maxAcceleration,
    units::angular_acceleration::radians_per_second_squared_t   maxAngularAcceleration
) : m_frontLeft(frontLeft), 
    m_frontRight(frontRight), 
    m_backLeft(backLeft), 
    m_backRight(backRight), 
    m_flState(),
    m_frState(),
    m_blState(),
    m_brState(),
    m_wheelDiameter(wheelDiameter),
    m_wheelBase(wheelBase),
    m_track(track),
    m_maxSpeed(maxSpeed),
    m_maxAngularSpeed(maxAngularSpeed), 
    m_maxAcceleration(maxAcceleration), //Not used at the moment
    m_maxAngularAcceleration(maxAngularAcceleration), //Not used at the moment
    m_drive(units::velocity::meters_per_second_t(0.0)),
    m_steer(units::velocity::meters_per_second_t(0.0)),
    m_rotate(units::angular_velocity::radians_per_second_t(0.0)),
    m_frontLeftLocation(wheelBase/2.0, track/2.0),
    m_frontRightLocation(wheelBase/2.0, -1.0*track/2.0),
    m_backLeftLocation(-1.0*wheelBase/2.0, track/2.0),
    m_backRightLocation(-1.0*wheelBase/2.0, -1.0*track/2.0)
{
    frontLeft.get()->Init( wheelDiameter, maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration, m_frontLeftLocation );
    frontRight.get()->Init( wheelDiameter, maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration, m_frontRightLocation );
    backLeft.get()->Init( wheelDiameter, maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration, m_backLeftLocation );
    backRight.get()->Init( wheelDiameter, maxSpeed, maxAngularSpeed, maxAcceleration, maxAngularAcceleration, m_backRightLocation );

    ZeroAlignSwerveModules();
}

void SwerveChassis::Drive
( 
    frc::ChassisSpeeds               speeds, 
    bool                        isFieldRelative
)
{
    auto xSpeed = speeds.vx; 
    auto ySpeed = speeds.vy; 
    auto rot = speeds.omega;

    //This if statement is probably needed, but should we declare the deadband in a different way? read from xml?
    /*if ( (abs(xSpeed.to<double>()) < m_deadband) && 
         (abs(ySpeed.to<double>()) < m_deadband) && 
         (abs(rot.to<double>())    < m_angularDeadband.to<double>()))
    {
        m_frontLeft.get()->StopMotors();
        m_frontRight.get()->StopMotors();
        m_backLeft.get()->StopMotors();
        m_backRight.get()->StopMotors();
        m_drive = units::velocity::meters_per_second_t(0.0);
        m_steer = units::velocity::meters_per_second_t(0.0);
        m_rotate = units::angular_velocity::radians_per_second_t(0.0);
    }
    else
    {*/   
        m_drive = units::velocity::meters_per_second_t(xSpeed);
        m_steer = units::velocity::meters_per_second_t(ySpeed);
        m_rotate = units::angular_velocity::radians_per_second_t(rot);

            frc::ChassisSpeeds chassisSpeeds = isFieldRelative ?
                                                    GetFieldRelativeSpeeds(xSpeed,ySpeed, rot) : 
                                                    frc::ChassisSpeeds{xSpeed, ySpeed, rot};
            auto states = m_kinematics.ToSwerveModuleStates(chassisSpeeds);
            m_kinematics.DesaturateWheelSpeeds(&states, m_maxSpeed);

            CalcSwerveModuleStates(chassisSpeeds);
            
            m_frontLeft.get()->SetDesiredState(m_flState);
            m_frontRight.get()->SetDesiredState(m_frState);
            m_backLeft.get()->SetDesiredState(m_blState);
            m_backRight.get()->SetDesiredState(m_brState);
    //}    
}

void SwerveChassis::Drive
( 
    double                      drive, 
    double                      steer, 
    double                      rotate, 
    bool                        isFieldRelative
)
{   
    // scale joystick values to velocities using max chassis values
    auto maxSpeed = GetMaxSpeed();
    auto maxRotation = GetMaxAngularSpeed();

    frc::ChassisSpeeds speeds;

    speeds.vx = drive*maxSpeed;
    speeds.vy = steer*maxSpeed;
    speeds.omega = rotate*maxRotation;

    //Just in case we get messed up speeds
    speeds.vx = speeds.vx > maxSpeed ? maxSpeed : speeds.vx;
    speeds.vy = speeds.vy > maxSpeed ? maxSpeed : speeds.vy;
    speeds.omega = speeds.omega > maxRotation ? maxRotation : speeds.omega;

    Drive(speeds, isFieldRelative);
}

void SwerveChassis::LogData()
{
    // Logger::GetLogger()->ToNtTable("Swerve Chassis", "XSpeed", xSpeed.to<double>() );
    // Logger::GetLogger()->ToNtTable("Swerve Chassis", "YSpeed", ySpeed.to<double>() );
    // Logger::GetLogger()->ToNtTable("Swerve Chassis", "ZSpeed", rot.to<double>() );
}

void SwerveChassis::CalcSwerveModuleStates
(
    frc::ChassisSpeeds speeds
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

    auto l = GetWheelBase();
    auto w = GetTrack();

    auto vy = 1.0 * speeds.vx;
    auto vx = -1.0 * speeds.vy;
    auto omega = speeds.omega;

    units::velocity::meters_per_second_t omegaL = omega.to<double>() * l / 2.0 / 1_s;
    units::velocity::meters_per_second_t omegaW = omega.to<double>() * w / 2.0 / 1_s;

    /// @TODO: Rework to allow for manipulating Center of Rotation to be inside and outside of robot.  Maybe have different calculations??
    
    auto a = vx - omegaL;
    auto b = vx + omegaL;
    auto c = vy - omegaW;
    auto d = vy + omegaW;

    // here we'll negate the angle to conform to the positive CCW convention
    m_flState.angle = units::angle::radian_t(atan2(b.to<double>(), d.to<double>()));
    m_flState.angle = -1.0 * m_flState.angle.Degrees();
    m_flState.speed = units::velocity::meters_per_second_t(sqrt( pow(b.to<double>(),2) + pow(d.to<double>(),2) ));
    auto maxCalcSpeed = abs(m_flState.speed.to<double>());

    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Front Left Angle", m_flState.angle.Degrees().to<double>());
    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Front Left Speed", m_flState.speed.to<double>());

    m_frState.angle = units::angle::radian_t(atan2(b.to<double>(), c.to<double>()));
    m_frState.angle = -1.0 * m_frState.angle.Degrees();
    m_frState.speed = units::velocity::meters_per_second_t(sqrt( pow(b.to<double>(),2) + pow(c.to<double>(),2) ));
    if (abs(m_frState.speed.to<double>())>maxCalcSpeed)
    {
        maxCalcSpeed = abs(m_frState.speed.to<double>());
    }

    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Front Right Angle", m_frState.angle.Degrees().to<double>());
    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Front Right Speed - raw", m_frState.speed.to<double>());

    m_blState.angle = units::angle::radian_t(atan2(a.to<double>(), d.to<double>()));
    m_blState.angle = -1.0 * m_blState.angle.Degrees();
    m_blState.speed = units::velocity::meters_per_second_t(sqrt( pow(a.to<double>(),2) + pow(d.to<double>(),2) ));
    if (abs(m_blState.speed.to<double>())>maxCalcSpeed)
    {
        maxCalcSpeed = abs(m_blState.speed.to<double>());
    }

    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Back Left Angle", m_blState.angle.Degrees().to<double>());
    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Back Left Speed - raw", m_blState.speed.to<double>());

    m_brState.angle = units::angle::radian_t(atan2(a.to<double>(), c.to<double>()));
    m_brState.angle = -1.0 * m_brState.angle.Degrees();
    m_brState.speed = units::velocity::meters_per_second_t(sqrt( pow(a.to<double>(),2) + pow(c.to<double>(),2) ));
    if (abs(m_brState.speed.to<double>())>maxCalcSpeed)
    {
        maxCalcSpeed = abs(m_brState.speed.to<double>());
    }

    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Back Right Angle", m_brState.angle.Degrees().to<double>());
    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Back Right Speed - raw", m_brState.speed.to<double>());


    // normalize speeds if necessary (maxCalcSpeed > max attainable speed)
    if ( maxCalcSpeed > m_maxSpeed.to<double>() )
    {
        auto ratio = m_maxSpeed.to<double>() / maxCalcSpeed;
        m_flState.speed *= ratio;
        m_frState.speed *= ratio;
        m_blState.speed *= ratio;
        m_brState.speed *= ratio;
    }

    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Front Left Speed - normalized", m_flState.speed.to<double>());
    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Front Right Speed - normalized", m_frState.speed.to<double>());
    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Back Left Speed - normalized", m_blState.speed.to<double>());
    Logger::GetLogger()->ToNtTable("Swerve Calcs", "Back Right Speed - normalized", m_brState.speed.to<double>());
}

/// @brief set all of the encoders to zero
void SwerveChassis::SetEncodersToZero()
{
    m_frontLeft.get()->SetEncodersToZero();
    m_frontRight.get()->SetEncodersToZero();
    m_backLeft.get()->SetEncodersToZero();
    m_backRight.get()->SetEncodersToZero();
}

double SwerveChassis::GetEncoderValues(std::shared_ptr<SwerveModule> motor)
{
    return motor.get()->GetEncoderValues();
}

/// @TODO: Add into SwerveUtils and implement a getter for m_kinematics

/// @brief Provide the current chassis speed information
/*ChassisSpeeds SwerveChassis::GetChassisSpeeds() const
{
    return m_kinematics.ToChassisSpeeds({ m_frontLeft.get()->GetState(), 
                                          m_frontRight.get()->GetState(),
                                          m_backLeft.get()->GetState(),
                                          m_backRight.get()->GetState() });
}*/