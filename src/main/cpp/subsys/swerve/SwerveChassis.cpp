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

/// @brief Drive the chassis
/// @param [in] frc::ChassisSpeeds  speeds:         kinematics for how to move the chassis
/// @param [in] CHASSIS_DRIVE_MODE  mode:           How the input chassis speeds are interpreted
/// @param [in] HEADING_OPTION      headingOption:  How the robot top should be facing
void SwerveChassis::Drive
( 
    ChassisSpeeds               speeds, 
    CHASSIS_DRIVE_MODE          mode,
    HEADING_OPTION              headingOption
)
{
    auto xSpeed = (abs(speeds.vx.to<double>()) < m_deadband) ? units::meters_per_second_t(0.0) : speeds.vx; 
    auto ySpeed = (abs(speeds.vy.to<double>()) < m_deadband) ? units::meters_per_second_t(0.0) : speeds.vy; 
    auto rot = (abs(speeds.omega.to<double>())) < m_angularDeadband.to<double>() ? units::radians_per_second_t(0.0) : speeds.omega;
    auto currentPose = GetPose();
    auto goalPose = m_targetFinder.GetPosCenterTarget();
    switch (headingOption)
    {
        case HEADING_OPTION::MAINTAIN:
             [[fallthrough]]; // intentional fallthrough 
        case HEADING_OPTION::POLAR_HEADING:
            AdjustRotToMaintainHeading(xSpeed, ySpeed, rot);
            break;

        case HEADING_OPTION::TOWARD_GOAL:
            AdjustRotToPointTowardGoal(currentPose, rot);
            Logger::GetLogger()->ToNtTable("Chassis Heading", "rot", rot.to<double>() );
            break;

        case HEADING_OPTION::TOWARD_GOAL_DRIVE:
             [[fallthrough]]; // intentional fallthrough 
        case HEADING_OPTION::TOWARD_GOAL_LAUNCHPAD:
            DriveToPointTowardGoal(currentPose,goalPose,xSpeed,ySpeed,rot);
            Logger::GetLogger()->ToNtTable("Chassis Heading", "rot", rot.to<double>() );
            break;

        case HEADING_OPTION::SPECIFIED_ANGLE:
            rot -= CalcHeadingCorrection(m_targetHeading, kPAutonSpecifiedHeading);
            Logger::GetLogger()->ToNtTable(string("Chassis Heading"), string("Specified Angle (Degrees): "), m_targetHeading.to<double>());
            Logger::GetLogger()->ToNtTable(string("Chassis Heading"), string("Heading Correctioin"), rot.to<double>());
            break;

        case HEADING_OPTION::LEFT_INTAKE_TOWARD_BALL:
            // TODO: implement
            break;

        case HEADING_OPTION::RIGHT_INTAKE_TOWARD_BALL:
            // TODO: implement
            break;

        default:
            break;
    }

    Logger::GetLogger()->ToNtTable("Swerve Chassis", "XSpeed", xSpeed.to<double>() );
    Logger::GetLogger()->ToNtTable("Swerve Chassis", "YSpeed", ySpeed.to<double>() );
    Logger::GetLogger()->ToNtTable("Swerve Chassis", "ZSpeed", rot.to<double>() );
    Logger::GetLogger()->ToNtTable("Swerve Chassis", "yaw", m_pigeon->GetYaw() );
    Logger::GetLogger()->ToNtTable("Swerve Chassis", "pitch", m_pigeon->GetPitch());
    Logger::GetLogger()->ToNtTable("Swerve Chassis", "angle error Degrees Per Second", m_yawCorrection.to<double>());

    Logger::GetLogger()->ToNtTable("Swerve Chassis", "Current X", GetPose().X().to<double>());
    Logger::GetLogger()->ToNtTable("Swerve Chassis", "Current Y", GetPose().Y().to<double>());
    Logger::GetLogger()->ToNtTable("Swerve Chassis", "Current Rot(Degrees)", GetPose().Rotation().Degrees().to<double>());
    
    if ( (abs(xSpeed.to<double>()) < m_deadband) && 
         (abs(ySpeed.to<double>()) < m_deadband) && 
         (abs(rot.to<double>())    < m_angularDeadband.to<double>()))  //our angular deadband, only used once, equates to 3 degrees per second
    {
        m_frontLeft.get()->StopMotors();
        m_frontRight.get()->StopMotors();
        m_backLeft.get()->StopMotors();
        m_backRight.get()->StopMotors();
        m_drive = units::velocity::meters_per_second_t(0.0);
        m_steer = units::velocity::meters_per_second_t(0.0);
        m_rotate = units::angular_velocity::radians_per_second_t(0.0);

        m_isMoving = false;
    }
    else
    {   
        m_drive = units::velocity::meters_per_second_t(xSpeed);
        m_steer = units::velocity::meters_per_second_t(ySpeed);
        m_rotate = units::angular_velocity::radians_per_second_t(rot);

        if ( m_runWPI )
        {
            units::degree_t yaw{m_pigeon->GetYaw()};
            Rotation2d currentOrientation {yaw};
            ChassisSpeeds chassisSpeeds = mode==IChassis::CHASSIS_DRIVE_MODE::FIELD_ORIENTED ? 
                                            ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, currentOrientation) : 
                                            ChassisSpeeds{xSpeed, ySpeed, rot};

            auto states = m_kinematics.ToSwerveModuleStates(chassisSpeeds);

            m_kinematics.DesaturateWheelSpeeds(&states, m_maxSpeed);

            auto [fl, fr, bl, br] = states;

            
            // adjust wheel angles
            if (mode == IChassis::CHASSIS_DRIVE_MODE::POLAR_DRIVE)
            {
                auto currentPose = GetPose();
                auto goalPose = m_targetFinder.GetPosCenterTarget();

                fr.angle = UpdateForPolarDrive(currentPose, goalPose, Transform2d(m_frontRightLocation, fr.angle), chassisSpeeds);
                bl.angle = UpdateForPolarDrive(currentPose, goalPose, Transform2d(m_backLeftLocation, bl.angle), chassisSpeeds);
                br.angle = UpdateForPolarDrive(currentPose, goalPose, Transform2d(m_backRightLocation, br.angle), chassisSpeeds);
                fl.angle = UpdateForPolarDrive(currentPose, goalPose, Transform2d(m_frontLeftLocation, fl.angle), chassisSpeeds);

                Logger::GetLogger()->ToNtTable("Polar Drive Calcs", "Front Left Angle", fl.angle.Degrees().to<double>());
                Logger::GetLogger()->ToNtTable("Polar Drive Calcs", "Front Right Angle", fr.angle.Degrees().to<double>());
                Logger::GetLogger()->ToNtTable("Polar Drive Calcs", "Back Left Angle", bl.angle.Degrees().to<double>());
                Logger::GetLogger()->ToNtTable("Polar Drive Calcs", "Back Right Angle", br.angle.Degrees().to<double>());
           }
        
            m_frontLeft.get()->SetDesiredState(fl);
            m_frontRight.get()->SetDesiredState(fr);
            m_backLeft.get()->SetDesiredState(bl);
            m_backRight.get()->SetDesiredState(br); 

            //Is moving check
            auto ax = m_accel.GetX();
            auto ay = m_accel.GetY();

            m_isMoving = (abs(ax) > 0.25 || abs(ay) > 0.25);
        }
        else
        {
            ChassisSpeeds chassisSpeeds = mode==IChassis::CHASSIS_DRIVE_MODE::FIELD_ORIENTED ?
                                                    GetFieldRelativeSpeeds(xSpeed,ySpeed, rot) : 
                                                    ChassisSpeeds{xSpeed, ySpeed, rot};
            auto states = m_kinematics.ToSwerveModuleStates(chassisSpeeds);
            m_kinematics.DesaturateWheelSpeeds(&states, m_maxSpeed);

            CalcSwerveModuleStates(chassisSpeeds);

            // adjust wheel angles
            if (mode == IChassis::CHASSIS_DRIVE_MODE::POLAR_DRIVE)
            {
                auto currentPose = GetPose();
                auto goalPose = m_targetFinder.GetPosCenterTarget();

                m_flState.angle = UpdateForPolarDrive(currentPose, goalPose, Transform2d(m_frontLeftLocation, m_flState.angle), chassisSpeeds);
                m_frState.angle = UpdateForPolarDrive(currentPose, goalPose, Transform2d(m_frontRightLocation, m_frState.angle), chassisSpeeds);
                m_blState.angle = UpdateForPolarDrive(currentPose, goalPose, Transform2d(m_backLeftLocation, m_blState.angle), chassisSpeeds);
                m_brState.angle = UpdateForPolarDrive(currentPose, goalPose, Transform2d(m_backRightLocation, m_brState.angle), chassisSpeeds);
           }

            auto ax = m_accel.GetX();
            auto ay = m_accel.GetY();
            auto az = m_accel.GetZ();

            m_isMoving = (abs(ax) > 0.0 || abs(ay) > 0.0 || abs(az) > 0.0);
            //TODO: Fix by removing az and tuning deadbands, will never be false because az returns 1G while not moving

            Logger::GetLogger()->ToNtTable("Swerve Chassis", "Hold Position State", m_hold);

            //Hold position / lock wheels in 'X' configuration
            if(m_hold && !frc::DriverStation::GetInstance().IsAutonomousEnabled() )
            {
                m_flState.angle = {units::angle::degree_t(45)};
                m_frState.angle = {units::angle::degree_t(-45)};
                m_blState.angle = {units::angle::degree_t(135)};
                m_brState.angle = {units::angle::degree_t(-135)};
            }
            
            m_frontLeft.get()->SetDesiredState(m_flState);
            m_frontRight.get()->SetDesiredState(m_frState);
            m_backLeft.get()->SetDesiredState(m_blState);
            m_backRight.get()->SetDesiredState(m_brState);
        }
    }    
}