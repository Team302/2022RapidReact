
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

// C++ Includes

// FRC includes
#include <wpi/array.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>

// Team 302 includes
#include <subsys/interfaces/ISwerveChassisModuleStates.h>
#include <subsys/interfaces/IChassis.h>

// Third Party Includes


namespace frc
{
    struct ChassisSpeeds;
    struct SwerveModuleState;
}


class WPIDirtySwerve : public ISwerveChassisModuleStates
{
	public:

        wpi::array<frc::SwerveModuleState, 4> CalcModuleStates
        (
            frc::ChassisSpeeds                      speeds
        ) override;


        WPIDirtySwerve() = delete;
        WPIDirtySwerve
        (
            units::length::meter_t                  wheelBase,
            units::length::meter_t                  wheelTrack,
            units::velocity::meters_per_second_t    maxSpeed
        );
        virtual ~WPIDirtySwerve() = default;

    private:
        units::length::meter_t                  m_wheelBase;
        units::length::meter_t                  m_wheelTrack;
        units::velocity::meters_per_second_t    m_maxSpeed;
        
        frc::Translation2d m_frontLeftLocation;
        frc::Translation2d m_frontRightLocation;
        frc::Translation2d m_backLeftLocation;
        frc::Translation2d m_backRightLocation;
        frc::SwerveDriveKinematics<4> m_kinematics{m_frontLeftLocation, 
                                                   m_frontRightLocation, 
                                                   m_backLeftLocation, 
                                                   m_backRightLocation};
};



