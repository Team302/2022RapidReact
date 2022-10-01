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
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <units/velocity.h>
#include <units/angular_velocity.h>

//Team302 Includes
#include <utils/DragonTargetFinder.h>
#include <hw/DragonLimelight.h>

//C++ Includes

class SwerveSpecial
{
    public:
        SwerveSpecial();

        /// @brief Tells the SwerveChassis whether to hold wheels in "X" defensive position
        /// @param [in] bool    holdState: Whether to hold the wheels or not
        void HoldPosition(bool holdState);

        /// @brief Drive to a point on the field
        /// @TODO: Change to make more general, have only changed name of function so far, not actual logic
        /// @param [in] frc::Pose2d robotPose:  Current pose of the robot
        /// @param [in] frc::Pose2d goalPose: Goal pose of the robot
        /// @param [in] units::meters_per_second_t  xSpeed: X-speed of the robot
        /// @param [in] units::meters_per_second_t  ySpeed: Y-speed of the robot
        /// @param [in] units::radians_per_second_t  zSpeed: Z-speed of the robot
        void DriveToPoint
        (
            frc::Pose2d              robotPose,
            frc::Pose2d              goalPose, 
            units::meters_per_second_t&  xspeed,
            units::meters_per_second_t&  yspeed,
            units::radians_per_second_t& rot
        );

        /// @brief Finds angle needed to rotate to face a specific point on the field
        /// @param [in] frc::Translation2d  point: The point on the field to turn towards
        /// @param [out] frc::Rotation2d - delta of robot angle and angle to face point
        frc::Rotation2d RotateToPoint(frc::Translation2d point);

        /// @TODO: Figure out how to actually do this.  Do we modify wheel angles and such, or do we output chassis speeds that circle the point?
        // For right now, it's set up for the ChassisSpeeds method
        /// @brief Output chassis speeds required to turn about a point
        /// @param [in] frc::Translation2d  point: The point on the field to turn towards
        /// @param [out] frc::ChassisSpeeds - speeds required to turn about point
        frc::ChassisSpeeds TurnAboutPoint(frc::Translation2d point);

        /// @brief Finds angle needed to rotate towards a vision target
        /// @param [out] frc::Rotation2d - delta of robot angle and angle to face vision target
        frc::Rotation2d TurnToVisionTarget();
    
    private:
        DragonTargetFinder m_targetFinder;
        units::angle::degree_t m_targetHeading;
        DragonLimelight*        m_limelight;
};