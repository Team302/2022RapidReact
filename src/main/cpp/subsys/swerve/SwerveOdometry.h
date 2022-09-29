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

//FIRST Includes
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/BuiltInAccelerometer.h>

//Team302 Includes
#include <hw/DragonPigeon.h>
#include <hw/factories/PigeonFactory.h>

#include <subsys/PoseEstimatorEnum.h>

//C++ Includes

class SwerveOdometry
{
    public:
        SwerveOdometry
        (

        );

    private:
        bool                                m_isMoving;

        DragonPigeon*                       m_pigeon;
        PoseEstimatorEnum                   m_poseOpt;

        frc::BuiltInAccelerometer           m_accel;
        frc::Pose2d                         m_pose;
        frc::SwerveDriveKinematics<4>       m_kinematics;
        frc::SwerveDrivePoseEstimator<4>    m_poseEstimator{  frc::Rotation2d(), 
                                                           frc::Pose2d(), 
                                                           m_kinematics,
                                                           {0.1, 0.1, 0.1},   // state standard deviations
                                                           {0.05},            // local measurement standard deviations
                                                           {0.1, 0.1, 0.1} }; // vision measurement standard deviations
};