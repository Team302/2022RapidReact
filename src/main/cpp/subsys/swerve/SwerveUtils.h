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

#pragma once

//FRC Includes
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>

#include <frc/BuiltInAccelerometer.h>

//Team302 Includes
#include <subsys/swerve/PoseEstimatorEnum.h>
#include <hw/DragonPigeon.h>
#include <hw/factories/PigeonFactory.h>



//C++ Includes

class SwerveUtils
{
    public:
        SwerveUtils(double odometryComplianceCoefficient);

        /// @brief update the chassis odometry based on current states of the swerve modules and the pigeon
        void UpdateOdometry();

        /// @brief Provide the current chassis speed information
        /// @TODO: Get chassis, then use old implementation here
        frc::ChassisSpeeds GetChassisSpeeds() const;

        /// @brief Reset the current chassis pose based on the provided pose and rotation
        /// @param [in] const Pose2d&       pose        Current XY position
        /// @param [in] const Rotation2d&   angle       Current rotation angle
        void ResetPosition
        ( 
            const frc::Pose2d&       pose,
            const frc::Rotation2d&   angle
        );

        /// @brief Reset the current chassis pose based on the provided pose (the rotation comes from the Pigeon)
        /// @param [in] const Pose2d&       pose        Current XY position
        void ResetPosition
        ( 
            const frc::Pose2d&       pose
        );

        /// @brief Get PoseEstimator for Swerve Drive
        frc::SwerveDrivePoseEstimator<4> GetPoseEst() const { return m_poseEstimator; }  

        /// @brief Get the current pose of the robot based on odometry
        /// @returns frc::Pose2d - Current Robot Pose
        frc::Pose2d GetPose() const;

        /// @brief Get current yaw, may be based on pigeon or odometry
        /// @returns units::angle::degree_t - Current Yaw
        units::angle::degree_t GetYaw() const;

        /// @brief Set the Pose Estimation Option
        /// @param [in] PoseEstimatorEnum   opt:    Which pose estimation method to use
        void SetPoseEstOption(PoseEstimatorEnum opt ) { m_poseOpt = opt; }

        /// @brief Returns if the robot is currently moving
        /// @returns bool - Is the robot moving?
        bool IsMoving() const { return m_isMoving;}

        /// @brief Get the Odometry Compliance Coefficient (Fancy fudge factor)
        /// @returns double - Odometry Compliance Coefficient
        double GetodometryComplianceCoefficient() const { return m_odometryComplianceCoefficient; }

        /// @TODO: Add functionality in SwerveDrive
        /// @brief Re-zero the odometry if robot is not driving field oriented
        void ReZero();

        /// @brief Calculate speed and position of swerve modules for field relative driving
        /// @param [in] units::meters_per_second_t  xSpeed: X speed of incoming chassis speeds
        /// @param [in] units::meters_per_second_t  ySpeed: Y speed of incoming chassis speeds
        /// @param [in] units::radians_per_second_t  rot: Z speed of incoming chassis speeds
        frc::ChassisSpeeds GetFieldRelativeSpeeds
        (
            units::meters_per_second_t xSpeed,
            units::meters_per_second_t ySpeed,
            units::radians_per_second_t rot        
        );

    private:
        double                                                      m_odometryComplianceCoefficient;

        DragonPigeon*                                               m_pigeon;
        frc::BuiltInAccelerometer                                   m_accel;
        bool                                                        m_isMoving;
        PoseEstimatorEnum                                           m_poseOpt;
        frc::Pose2d                                                 m_pose;

        SwerveChassis*                                              m_chassis;

        // Gains are for example purposes only - must be determined for your own robot!
        //Clean up to get clearer information
        frc::SwerveDriveKinematics<4> m_kinematics;

        frc::SwerveDrivePoseEstimator<4> m_poseEstimator{  frc::Rotation2d(), 
                                                           frc::Pose2d(), 
                                                           m_kinematics,
                                                           {0.1, 0.1, 0.1},   // state standard deviations
                                                           {0.05},            // local measurement standard deviations
                                                           {0.1, 0.1, 0.1} }; // vision measurement standard deviations
        const double kPMaintainHeadingControl = 2.0; //was 1.5 7/25/22
        const double kPAutonSpecifiedHeading = 3.0;  // 4.0
        const double kPAutonGoalHeadingControl = 5.0;  // 2.0
        const double kPGoalHeadingControl = 6.0; //10.0, 7.0
        const double kPDistance = 10.0; //10.0, 7.0
        const double kIHeadingControl = 0.0; //not being used
        const double kDHeadingControl = 0.0; //not being used
        const double kFHeadingControl = 0.0; //not being used
        bool m_hold = false;
        units::angle::degree_t m_storedYaw;

        std::shared_ptr<SwerveModule>                               m_frontLeft;
        std::shared_ptr<SwerveModule>                               m_frontRight;
        std::shared_ptr<SwerveModule>                               m_backLeft;
        std::shared_ptr<SwerveModule>                               m_backRight;        
};