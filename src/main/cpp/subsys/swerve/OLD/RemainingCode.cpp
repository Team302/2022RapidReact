
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
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include <frc/drive/Vector2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>

#include <frc/DriverStation.h>

#include <wpi/numbers>

// Team 302 includes
#include <hw/DragonLimelight.h>
#include <hw/factories/LimelightFactory.h>
#include <subsys/PoseEstimatorEnum.h>
#include <subsys/swerve/OLD/OLDSwerveChassis.h>
#include <utils/AngleUtils.h>
#include <utils/Logger.h>
#include <gamepad/TeleopControl.h>

// Third Party Includes
#include <ctre/phoenix/sensors/CANCoder.h>

#include <iostream>

using namespace std;
using namespace frc;

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
    
) : 
    m_storedYaw(m_pigeon->GetYaw()),
    m_yawCorrection(units::angular_velocity::degrees_per_second_t(0.0)),
    m_targetHeading(units::angle::degree_t(0)),
    m_limelight(LimelightFactory::GetLimelightFactory()->GetLimelight())
{
}


units::angular_velocity::degrees_per_second_t SwerveChassis::CalcHeadingCorrection
(
    units::angle::degree_t  targetAngle,
    double                  kP
) 
{
    auto currentAngle = GetPose().Rotation().Degrees();
    auto errorAngle = AngleUtils::GetEquivAngle(AngleUtils::GetDeltaAngle(currentAngle, targetAngle));
    auto correction = units::angular_velocity::degrees_per_second_t(errorAngle.to<double>()*kP);

    //Debugging
    Logger::GetLogger()->ToNtTable("Chassis Heading", "Current Angle (Degrees): ", currentAngle.to<double>());
    Logger::GetLogger()->ToNtTable("Chassis Heading", "Error Angle (Degrees): ", errorAngle.to<double>());
    Logger::GetLogger()->ToNtTable("Chassis Heading", "Yaw Correction (Degrees Per Second): ", m_yawCorrection.to<double>());

    return correction;
}



void SwerveChassis::HoldPosition(bool holdState)
{
    m_hold = holdState;
}

units::angle::degree_t SwerveChassis::UpdateForPolarDrive
(
    Pose2d              robotPose,
    Pose2d              goalPose,
    Transform2d         wheelLoc,
    ChassisSpeeds       speeds
)
{
    Transform2d relativeWheelPosition = wheelLoc;
    //This wheel pose may not be accurate, may need to do manually using trig functions
    auto tempRobotPose = robotPose;
    tempRobotPose.Rotation().Degrees() - units::degree_t(0.0);
    auto WheelPose = tempRobotPose + relativeWheelPosition;

    auto wheelDeltaX = WheelPose.X() - goalPose.X();
    auto wheelDeltaY = WheelPose.Y() - goalPose.Y();

    Rotation2d ninety {units::angle::degree_t(-90.0)};

    //Change angle to change direction of wheel based on quadrant
    if (m_targetFinder.GetFieldQuadrant(WheelPose) == 1 || m_targetFinder.GetFieldQuadrant(WheelPose) == 3)
    {
        ninety.Degrees() = units::angle::degree_t(90.0); //Might have to switch signs
    }
    else if (m_targetFinder.GetFieldQuadrant(WheelPose) == 2 || m_targetFinder.GetFieldQuadrant(WheelPose) == 4)
    {
        ninety.Degrees() = units::angle::degree_t(-90.0);
    }

    units::angle::radian_t triangleThetaRads = units::angle::radian_t(atan(wheelDeltaY.to<double>() / wheelDeltaX.to<double>()));
    units::angle::degree_t thetaDeg = triangleThetaRads; //- robotPose.Rotation().Degrees(); Subtract robot pose to "normalize" wheels, zero for the wheels is the robot angle

    //Debugging
    Logger::GetLogger()->ToNtTable("Polar Drive Calcs", "WheelPoseX (Meters)", WheelPose.X().to<double>());
    Logger::GetLogger()->ToNtTable("Polar Drive Calcs", "WheelPoseY (Meters)", WheelPose.Y().to<double>());
    Logger::GetLogger()->ToNtTable("Polar Drive Calcs", "WheelDeltaX (Meters)", wheelDeltaX.to<double>());
    Logger::GetLogger()->ToNtTable("Polar Drive Calcs", "WheelDeltaY (Meters)", wheelDeltaY.to<double>());
    Logger::GetLogger()->ToNtTable("Polar Drive Calcs", "Triangle Theta", thetaDeg.to<double>());
    Logger::GetLogger()->ToNtTable("Polar Drive Calcs", "Ninety (Degrees)", ninety.Degrees().to<double>());
    Logger::GetLogger()->ToNtTable("Polar Drive Calcs", "Field Quadrant", m_targetFinder.GetFieldQuadrant(WheelPose));

    auto radialAngle = thetaDeg;
    auto orbitAngle = thetaDeg + ninety.Degrees();

    Logger::GetLogger()->ToNtTable("Polar Drive Calcs", "Orbit Angle (Degrees)", orbitAngle.to<double>());
    Logger::GetLogger()->ToNtTable("Polar Drive Calcs", "Radial Angle (Degrees)", radialAngle.to<double>());

    auto hasRadialComp = (abs(speeds.vx.to<double>()) > 0.1);
    auto hasOrbitComp = (abs(speeds.vy.to<double>()) > 0.1);

    if (hasRadialComp && !hasOrbitComp)
    {
        return radialAngle;
    }
    else if (!hasRadialComp && hasOrbitComp)
    {
        return orbitAngle;
    }
    else if (hasRadialComp && hasOrbitComp)
    {
        auto radialPercent = (speeds.vx / speeds.vy);
        auto orbitPercent  = (speeds.vy / speeds.vx);
        return ((radialPercent * radialAngle) + (orbitPercent * orbitAngle));
    }
    else
    {
        return 0_deg;
    }
}

/// @brief Drive the chassis
/// @param [in] double  drivePercent:   forward/reverse percent output (positive is forward)
/// @param [in] double  steerPercent:   left/right percent output (positive is left)
/// @param [in] double  rotatePercent:  Rotation percent output around the vertical (Z) axis; (positive is counter clockwise)
/// @param [in] bool    fieldRelative:  true: movement is based on the field (e.g., push it goes away from the driver regardless of the robot orientation),
///                                     false: direction is based on robot front/back
/// @param [in] bool    useTargetHeading: true: constrain the heading based on the stored target heading,
///                                     false: don't contrain the heading
void SwerveChassis::Drive
( 
    double                      drive, 
    double                      steer, 
    double                      rotate, 
    CHASSIS_DRIVE_MODE          mode,
    HEADING_OPTION              headingOption
)
{
    if ( abs(drive)  < m_deadband && 
         abs(steer)  < m_deadband && 
         abs(rotate) < m_deadband)
    {
        // feed the motors
        m_frontLeft.get()->StopMotors();
        m_frontRight.get()->StopMotors();
        m_backLeft.get()->StopMotors();
        m_backRight.get()->StopMotors();       
    }
    else
    {    
        // scale joystick values to velocities using max chassis values
        auto maxSpeed = GetMaxSpeed();
        auto maxRotation = GetMaxAngularSpeed();

        ChassisSpeeds speeds;
        speeds.vx = drive*maxSpeed;
        speeds.vy = steer*maxSpeed;
        speeds.omega = rotate*maxRotation;

        //Just in case we get messed up speeds
        speeds.vx = speeds.vx > maxSpeed ? maxSpeed : speeds.vx;
        speeds.vy = speeds.vy > maxSpeed ? maxSpeed : speeds.vy;
        speeds.omega = speeds.omega > maxRotation ? maxRotation : speeds.omega;

        Drive(speeds, mode, headingOption);
    }
}

void SwerveChassis::AdjustRotToMaintainHeading
(
    units::meters_per_second_t&  xspeed,
    units::meters_per_second_t&  yspeed,
    units::radians_per_second_t& rot 
)
{
    Logger::GetLogger()->ToNtTable("SwerveChassis", "RotBeforeMaintain", rot.to<double>());
    units::angular_velocity::degrees_per_second_t correction = units::angular_velocity::degrees_per_second_t(0.0);
    if (abs(rot.to<double>()) < 0.2)
    {
        rot = units::radians_per_second_t(0.0);
        if (abs(xspeed.to<double>()) > 0.0 || abs(yspeed.to<double>() > 0.0))
        {
            correction = CalcHeadingCorrection(m_storedYaw, kPMaintainHeadingControl);
        }
    }
    else
    {
        m_storedYaw = GetPose().Rotation().Degrees();
    }
    rot -= correction;
    Logger::GetLogger()->ToNtTable("SwerveChassis", "RotAfterMaintain Radians Per Second", rot.to<double>());
    Logger::GetLogger()->ToNtTable("SwerveChassis", "Stored Yaw Degrees", m_storedYaw.to<double>());
    Logger::GetLogger()->ToNtTable("SwerveChassis", "Correction Degrees Per Second", correction.to<double>());
}

void SwerveChassis::DriveToPointTowardGoal
(   
    Pose2d                     robotPose,
    Pose2d                     goalPose, 
    units::meters_per_second_t &xSpeed,
    units::meters_per_second_t &ySpeed,
    units::radians_per_second_t &rot     
)
{
    auto myPose = robotPose;
    auto targetPose = goalPose;
    frc::Pose2d driveToPose;

    auto distanceError = m_shootingDistance - m_limelight->EstimateTargetDistance();

    //Finding Target pose on feild based on current position
    double theta = abs(atan((targetPose.X()-myPose.X()).to<double>()/((targetPose.Y()-myPose.Y()).to<double>())));
    double xComp = sin(theta)*(m_limelight->EstimateTargetDistance().to<double>() + 24.0)*0.0254;//adding 24 inches offset for the center of goal, converting to meters
    double yComp = cos(theta)*(m_limelight->EstimateTargetDistance().to<double>() + 24.0)*0.0254;//adding 24 inches offset for the center of goal, converting to meters

    double speedCorrection = (distanceError.to<double>() < 30.0) ? kPDistance*2.0 : kPDistance;

    if (m_limelight != nullptr && m_limelight->HasTarget())
    { 
        if (abs(distanceError.to<double>()) > 10.0)
        {
            AdjustRotToPointTowardGoal(robotPose, rot);

            //adding or subrtacting deltaX/deltay based on quadrant 
            //  What quadruarnt is the robot in based on center of target      +=Center Target
            //                  |
            //               II |   I
            //             -----+------
            //              III |   IV
            //                  |

            if((targetPose.X()-myPose.X()).to<double>()  <= 0 && (targetPose.Y()-myPose.Y()).to<double>()  >= 0)//Quad 1
            {
                driveToPose = frc::Pose2d(targetPose.X() + units::length::meter_t{xComp}, targetPose.Y() - units::length::meter_t{yComp},0_deg);   
            }   
            else if((targetPose.X()-myPose.X()).to<double>()  <= 0 && (targetPose.Y()-myPose.Y()).to<double>()  <= 0)//Quad 2
            {
                driveToPose = frc::Pose2d(targetPose.X() + units::length::meter_t{xComp}, targetPose.Y() + units::length::meter_t{yComp},0_deg);  
            }
            else if((targetPose.X()-myPose.X()).to<double>() >= 0 && (targetPose.Y()-myPose.Y()).to<double>() >= 0)//Quad 3
            {
                driveToPose = frc::Pose2d(targetPose.X() - units::length::meter_t{xComp}, targetPose.Y() + units::length::meter_t{yComp},0_deg);  
            }
            else //Quad 4
            {
                driveToPose = frc::Pose2d(targetPose.X() - units::length::meter_t{xComp}, targetPose.Y() - units::length::meter_t{yComp},0_deg); 
            }
            auto deltaX = (driveToPose.X()-myPose.X());
            auto deltaY = (driveToPose.Y()-myPose.Y());
            xSpeed += deltaX/1_s*speedCorrection; 
            ySpeed += deltaY/1_s*speedCorrection; 

            m_hold = false;
        }
        else
        {
            AdjustRotToPointTowardGoal(robotPose, rot);
            m_hold = false;
        }
    }
    else
    {
        AdjustRotToPointTowardGoal(robotPose, rot);
    }
    m_storedYaw = GetPose().Rotation().Degrees();
    Logger::GetLogger()->ToNtTable(string("Chassis Heading"), string("TurnToGoal New ZSpeed: "), rot.to<double>());
}

void SwerveChassis::AdjustRotToPointTowardGoal
(   
    Pose2d                      robotPose,
    units::radians_per_second_t &rot     
)
{
    if(abs(m_limelight->GetTargetHorizontalOffset().to<double>()) < 1.0 && m_limelight->HasTarget())
    {
        m_hold = true;
    }
    else if (m_limelight != nullptr && m_limelight->HasTarget())
    { 
        double rotCorrection = abs(m_limelight->GetTargetHorizontalOffset().to<double>()) > 10.0 ? kPGoalHeadingControl : kPGoalHeadingControl*2.0;
        rot += (m_limelight->GetTargetHorizontalOffset())/1_s*rotCorrection;
        m_hold = false;   
    }
    else
    {
        auto targetAngle = units::angle::degree_t(m_targetFinder.GetTargetAngleD(robotPose));
        rot -= CalcHeadingCorrection(targetAngle,kPGoalHeadingControl);
        m_hold = false;
    }

    m_storedYaw = GetPose().Rotation().Degrees();

    Logger::GetLogger()->ToNtTable(string("Chassis Heading"), string("TurnToGoal New ZSpeed: "), rot.to<double>());
}

Pose2d SwerveChassis::GetPose() const
{
    if (m_poseOpt==PoseEstimatorEnum::WPI)
    {
        return m_poseEstimator.GetEstimatedPosition();
    }
    return m_pose;
}

units::angle::degree_t SwerveChassis::GetYaw() const
{
    units::degree_t yaw{m_pigeon->GetYaw()};
    return yaw;
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


/// @brief Provide the current chassis speed information
ChassisSpeeds SwerveChassis::GetChassisSpeeds() const
{
    return m_kinematics.ToChassisSpeeds({ m_frontLeft.get()->GetState(), 
                                          m_frontRight.get()->GetState(),
                                          m_backLeft.get()->GetState(),
                                          m_backRight.get()->GetState() });
}

/// @brief Reset the current chassis pose based on the provided pose and rotation
/// @param [in] const Pose2d&       pose        Current XY position
/// @param [in] const Rotation2d&   angle       Current rotation angle
void SwerveChassis::ResetPosition
( 
    const Pose2d&       pose,
    const Rotation2d&   angle
)
{
    m_poseEstimator.ResetPosition(pose, angle);
    SetEncodersToZero();
    m_pose = pose;

    auto pigeon = PigeonFactory::GetFactory()->GetPigeon(DragonPigeon::PIGEON_USAGE::CENTER_OF_ROBOT);

    pigeon->ReZeroPigeon(angle.Degrees().to<double>(), 0);

    m_storedYaw = angle.Degrees();

    //m_offsetPoseAngle = units::angle::degree_t(m_pigeon->GetYaw()) - angle.Degrees();

    Transform2d t_fl {m_frontLeftLocation,angle};
    auto flPose = pose + t_fl;
    m_frontLeft.get()->UpdateCurrPose(flPose.X(), flPose.Y());

    Transform2d t_fr {m_frontRightLocation,angle};
    auto frPose = m_pose + t_fr;
    m_frontRight.get()->UpdateCurrPose(frPose.X(), frPose.Y());

    Transform2d t_bl {m_backLeftLocation,angle};
    auto blPose = m_pose + t_bl;
    m_backLeft.get()->UpdateCurrPose(blPose.X(), blPose.Y());

    Transform2d t_br {m_backRightLocation,angle};
    auto brPose = m_pose + t_br;
    m_backRight.get()->UpdateCurrPose(brPose.X(), brPose.Y());
}


void SwerveChassis::ResetPosition
( 
    const Pose2d&       pose
)
{
    Rotation2d angle = pose.Rotation();

    ResetPosition(pose, angle);
}

void SwerveChassis::SetTargetHeading(units::angle::degree_t targetYaw) 
{
    m_targetHeading = targetYaw;
}

void SwerveChassis::ReZero()
{
    m_storedYaw = units::angle::degree_t(0.0);
}