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

//FIRST Includes


//Team302 Includes
#include <subsys/swerve/SwerveUtils.h>
#include <subsys/swerve/SwerveChassis.h>

#include <subsys/ChassisFactory.h>

#include <utils/Logger.h>

//C++ Includes

SwerveUtils::SwerveUtils
(
    double                                                      odometryComplianceCoefficient
) : m_odometryComplianceCoefficient(odometryComplianceCoefficient),
    m_pigeon(PigeonFactory::GetFactory()->GetPigeon(DragonPigeon::PIGEON_USAGE::CENTER_OF_ROBOT)),
    m_accel(frc::BuiltInAccelerometer()),
    m_isMoving(false),
    m_poseOpt(PoseEstimatorEnum::WPI),
    m_pose(),
    m_chassis(ChassisFactory::GetChassisFactory()->GetSwerveChassis()),
    m_kinematics(m_chassis->GetSwerveKinematics()),
    m_storedYaw(m_pigeon->GetYaw()),
    m_frontLeft(m_chassis->GetFrontLeft()),
    m_frontRight(m_chassis->GetFrontRight()),
    m_backLeft(m_chassis->GetBackLeft()),
    m_backRight(m_chassis->GetBackRight())

{

}


frc::ChassisSpeeds SwerveUtils::GetFieldRelativeSpeeds
(
    units::meters_per_second_t xSpeed,
    units::meters_per_second_t ySpeed,
    units::radians_per_second_t rot        
)
{
    Logger::GetLogger()->ToNtTable("Field Oriented Calcs", "xSpeed (mps)", xSpeed.to<double>());
    Logger::GetLogger()->ToNtTable("Field Oriented Calcs", "ySpeed (mps)", ySpeed.to<double>());
    Logger::GetLogger()->ToNtTable("Field Oriented Calcs", "rot (radians per sec)", rot.to<double>());

    units::angle::radian_t yaw{m_pigeon->GetYaw()*wpi::numbers::pi/180.0};
    auto forward = xSpeed*cos(yaw.to<double>()) + ySpeed*sin(yaw.to<double>());
    auto strafe = -1.0 *xSpeed*sin(yaw.to<double>()) + ySpeed*cos(yaw.to<double>());

    frc::ChassisSpeeds output{forward, strafe, rot};

    Logger::GetLogger()->ToNtTable("Field Oriented Calcs", "yaw (radians)", yaw.to<double>());
    Logger::GetLogger()->ToNtTable("Field Oriented Calcs", "forward (mps)", forward.to<double>());
    Logger::GetLogger()->ToNtTable("Field Oriented Calcs", "stafe (mps)", strafe.to<double>());

    return output;
}


void SwerveUtils::UpdateOdometry() 
{
    units::degree_t yaw{m_pigeon->GetYaw()};
    frc::Rotation2d rot2d {yaw}; //used to add m_offsetAngle but now we update pigeon yaw in ResetPosition.cpp

    if (m_poseOpt == PoseEstimatorEnum::WPI)
    {
        auto currentPose = m_poseEstimator.GetEstimatedPosition();
        Logger::GetLogger()->ToNtTable("Robot Odometry", "Current X", currentPose.X().to<double>());
        Logger::GetLogger()->ToNtTable("Robot Odometry", "Current Y", currentPose.Y().to<double>());

        m_poseEstimator.Update(rot2d, m_frontLeft->GetState(),
                                      m_frontRight->GetState(), 
                                      m_backLeft->GetState(),
                                      m_backRight->GetState());

        auto updatedPose = m_poseEstimator.GetEstimatedPosition();
        Logger::GetLogger()->ToNtTable("Robot Odometry", "Updated X", updatedPose.X().to<double>());
        Logger::GetLogger()->ToNtTable("Robot Odometry", "Updated Y", updatedPose.Y().to<double>());
    }

    /// @TODO:  Are these other pose estimation options needed?

    /*
    else if (m_poseOpt==PoseEstimatorEnum::EULER_AT_CHASSIS)
    {
        // get change in time
        auto deltaT = m_timer.Get();
        m_timer.Reset();

        // get the information from the last pose 
        auto startX = m_pose.X();
        auto startY = m_pose.Y();

        // xk+1 = xk + vk cos θk T
        // yk+1 = yk + vk sin θk T
        // Thetak+1 = Thetagyro,k+1
        units::angle::radian_t rads = yaw;          // convert angle to radians
        double cosAng = cos(rads.to<double>());
        double sinAng = sin(rads.to<double>());
        auto vx = m_drive * cosAng + m_steer * sinAng;
        auto vy = m_drive * sinAng + m_steer * cosAng;

        units::length::meter_t currentX = startX + m_odometryComplianceCoefficient*(vx * deltaT);
        units::length::meter_t currentY = startY + m_odometryComplianceCoefficient*(vy * deltaT);

        frc::Pose2d currPose{currentX, currentY, rot2d};
        auto trans = currPose - m_pose;
        m_pose = m_pose + trans;
    }
    else if (m_poseOpt==PoseEstimatorEnum::EULER_USING_MODULES ||
             m_poseOpt==PoseEstimatorEnum::POSE_EST_USING_MODULES)
    {
        auto flPose = m_frontLeft.get()->GetCurrentPose(m_poseOpt);
        auto frPose = m_frontRight.get()->GetCurrentPose(m_poseOpt);
        auto blPose = m_backLeft.get()->GetCurrentPose(m_poseOpt);
        auto brPose = m_backRight.get()->GetCurrentPose(m_poseOpt);

        auto chassisX = (flPose.X() + frPose.X() + blPose.X() + brPose.X()) / 4.0;
        auto chassisY = (flPose.Y() + frPose.Y() + blPose.Y() + brPose.Y()) / 4.0;
        frc::Pose2d currPose{chassisX, chassisY, rot2d};
        auto trans = currPose - m_pose;
        m_pose = m_pose + trans;
    }
    */
}