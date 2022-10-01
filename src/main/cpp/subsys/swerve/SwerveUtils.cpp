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
    m_kinematics(SwerveChassis::GetSwerveKinematics())
{

}


ChassisSpeeds SwerveChassis::GetFieldRelativeSpeeds
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

    ChassisSpeeds output{forward, strafe, rot};

    Logger::GetLogger()->ToNtTable("Field Oriented Calcs", "yaw (radians)", yaw.to<double>());
    Logger::GetLogger()->ToNtTable("Field Oriented Calcs", "forward (mps)", forward.to<double>());
    Logger::GetLogger()->ToNtTable("Field Oriented Calcs", "stafe (mps)", strafe.to<double>());

    return output;
}