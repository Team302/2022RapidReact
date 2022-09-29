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

//Team302 Includes
#include <subsys/swerve/SwerveModule.h>
#include <subsys/interfaces/IChassis.h>

class SwerveChassis : public IChassis
{
    public:

        /// @brief Construct a swerve chassis
        /// @param [in] std::shared_ptr<SwerveModule>           frontleft:          front left swerve module
        /// @param [in] std::shared_ptr<SwerveModule>           frontright:         front right swerve module
        /// @param [in] std::shared_ptr<SwerveModule>           backleft:           back left swerve module
        /// @param [in] std::shared_ptr<SwerveModule>           backright:          back right swerve module
        /// @param [in] units::length::inch_t                   wheelDiameter:      Diameter of the wheel
        /// @param [in] units::length::inch_t                   wheelBase:          distance between the front and rear wheels
        /// @param [in] units::length::inch_t                   track:              distance between the left and right wheels
        /// @param [in] units::velocity::meters_per_second_t    maxSpeed:           maximum linear speed of the chassis 
        /// @param [in] units::radians_per_second_t             maxAngularSpeed:    maximum rotation speed of the chassis 
        /// @param [in] double                                  maxAcceleration:    maximum acceleration in meters_per_second_squared
        
        SwerveChassis
        ( 
			std::shared_ptr<SwerveModule>                               frontLeft, 
			std::shared_ptr<SwerveModule>                               frontRight,
			std::shared_ptr<SwerveModule>                               backLeft, 
			std::shared_ptr<SwerveModule>                               backRight, 
            units::length::inch_t                                       wheelDiameter,
			units::length::inch_t                                       wheelBase,
			units::length::inch_t                                       track,
            double                                                      odometryComplianceCoefficient,
			units::velocity::meters_per_second_t                        maxSpeed,
			units::radians_per_second_t                                 maxAngularSpeed,
			units::acceleration::meters_per_second_squared_t            maxAcceleration,
			units::angular_acceleration::radians_per_second_squared_t   maxAngularAcceleration
        );

        /// @brief Align all of the swerve modules to point forward
        void ZeroAlignSwerveModules();

        /// @brief Drive the chassis
        /// @param [in] double  drivePercent:   forward/reverse percent output (positive is forward)
        /// @param [in] double  steerPercent:   left/right percent output (positive is left)
        /// @param [in] double  rotatePercent:  Rotation percent output around the vertical (Z) axis; (positive is counter clockwise)
        /// @param [in] bool    fieldRelative:  true: movement is based on the field (e.g., push it goes away from the driver regardless of the robot orientation),
        ///                                     false: direction is based on robot front/back
        void Drive
        (
            double drivePercent, 
            double steerPercent, 
            double rotatePercent, 
            bool   isFieldRelative
        );

        /// @brief Drive the chassis
        /// @param [in] frc::ChassisSpeeds  speeds:         kinematics for how to move the chassis
        /// @param [in] bool                fieldRelative:  true: movement is based on the field (e.g., push it goes away from the driver regardless of the robot orientation),
        ///                                                 false: direction is based on robot front/back
        void Drive
        (
            frc::ChassisSpeeds speeds, 
            bool isFieldRelative
        ) override;

        /// @brief Sets of the motor encoders to zero
        void SetEncodersToZero();

        /// @brief Get encoder values
        double GetEncoderValues(std::shared_ptr<SwerveModule> motor);

        units::length::inch_t GetWheelDiameter() const {return m_wheelDiameter; }  
        units::length::inch_t GetWheelBase() const {return m_wheelBase; }  
        units::length::inch_t GetTrack() const {return m_track;}
        units::velocity::meters_per_second_t GetMaxSpeed() const {return m_maxSpeed;}
        units::radians_per_second_t GetMaxAngularSpeed() const {return m_maxAngularSpeed;}
        units::acceleration::meters_per_second_squared_t GetMaxAcceleration() const { return m_maxAcceleration; }
        units::angular_acceleration::radians_per_second_squared_t GetMaxAngularAcceleration() const { return m_maxAngularAcceleration; }
        std::shared_ptr<SwerveModule> GetFrontLeft() const { return m_frontLeft;}
        std::shared_ptr<SwerveModule> GetFrontRight() const { return m_frontRight;}
        std::shared_ptr<SwerveModule> GetBackLeft() const { return m_backLeft;}
        std::shared_ptr<SwerveModule> GetBackRight() const { return m_backRight;}

        //Dummy functions for IChassis Implementation
        inline IChassis::CHASSIS_TYPE GetType() const override {return IChassis::CHASSIS_TYPE::SWERVE;};
        inline void Initialize() override {};

    private:
        /// @brief Calculate state of swerve modules based on incoming ChassisSpeeds
        /// @param [in] frc::ChassisSpeeds  
        void CalcSwerveModuleStates
        (
            frc::ChassisSpeeds 
        );



}