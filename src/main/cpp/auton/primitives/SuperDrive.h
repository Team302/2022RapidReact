
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
#include <memory>

// FRC includes
#include <frc/kinematics/DifferentialDriveKinematics.h>

// Team 302 includes
#include <auton/primitives/IPrimitive.h>

// Third Party Includes


class IChassis;
namespace frc
{
	class Timer;
}


class SuperDrive : public IPrimitive 
{
	public:
		void Init(PrimitiveParams* params) override;
		void Run() override;
		bool IsDone() override;
		void SlowDown();
		bool ReachedTargetSpeed();

		const double GYRO_CORRECTION_CONSTANT = 0.001;//0.1//6; //2.3
		const double INCHES_PER_SECOND_SECOND = 120; //120
		const double MIN_SPEED_SLOWDOWN       = 13;

protected: 
		SuperDrive();
		virtual ~SuperDrive() = default;

	private:
		const double PROPORTIONAL_COEFF  = 12.0; //16
		const double INTREGRAL_COEFF     = 0;
		const double DERIVATIVE_COEFF    = 0.0; //.16
		const double FEET_FORWARD_COEFF  = 0.0;

        std::shared_ptr<IChassis> m_chassis;
   		std::unique_ptr<frc::Timer> m_timer;

		double m_targetSpeed;
		double m_currentSpeed;
		double m_speedOffset;

		double m_leftSpeed;
		double m_rightSpeed;

		double m_currentHeading;
		double m_startHeading;

		bool m_slowingDown;
		bool m_reachedTargetSpeed;
		double m_accelDecelTime;
		double m_currentTime;
		double m_minSpeedSlowdown;
		frc::DifferentialDriveKinematics* m_kinematics;
};

