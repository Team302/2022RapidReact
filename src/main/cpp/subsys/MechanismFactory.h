
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

//========================================================================================================
/// MechanismFactory.h
//========================================================================================================
///
/// File Description:
///     This controls the creation of mechanisms/subsystems
///
//========================================================================================================

#pragma once

// C++ Includes
#include <map>
#include <memory>
#include <vector>

// FRC includes

// Team 302 includes
#include <hw/usages/AnalogInputMap.h>
#include <hw/usages/DigitalInputMap.h>
#include <hw/usages/DragonSolenoidMap.h>
#include <hw/usages/IDragonMotorControllerMap.h>
#include <hw/usages/ServoMap.h>
#include <subsys/Climber.h>
#include <subsys/Indexer.h>
#include <subsys/Intake.h>
#include <subsys/interfaces/IMech.h>
#include <subsys/Lift.h>
#include <subsys/MechanismTypes.h>
#include <subsys/Shooter.h>

// Third Party Includes

// forward declares
class DragonAnalogInput;
class DragonDigitalInput;
class DragonServo;
class DragonSolenoid;
class IDragonMotorController;
class IMech;

namespace ctre
{
	namespace phoenix
	{
		namespace sensors
		{
			class CANCoder;
		}
	}
}


class MechanismFactory
{
	public:

		static MechanismFactory* GetMechanismFactory();


		//=====================================================================================
		/// Method:         CreateIMechanism
		/// Description:    Find or create the requested mechanism
		//=====================================================================================
		void  CreateIMechanism
		(
			MechanismTypes::MECHANISM_TYPE							type,
			std::string												networkTableName,
			std::string												controlFileName,
			const IDragonMotorControllerMap&        				motorControllers,   // <I> - Motor Controllers
			const DragonSolenoidMap&                				solenoids,
			const ServoMap&						    				servos,
			const DigitalInputMap&									digitalInputs,
			const AnalogInputMap& 								    analogInputs,
			std::shared_ptr<ctre::phoenix::sensors::CANCoder>		canCoder
		);
		
		inline Intake* GetLeftIntake() const { return m_leftIntake;};
		inline Intake* GetRightIntake() const { return m_rightIntake;};
		inline Shooter* GetShooter() const { return m_shooter;};
		inline Indexer* GetIndexer() const { return m_indexer;};
		inline Lift* GetLift() const { return m_lift;};

		IMech* GetMechanism
		(
			MechanismTypes::MECHANISM_TYPE	type
		) const;

		inline Climber* GetClimber() const {return m_climber;}

	private:
		std::shared_ptr<IDragonMotorController> GetMotorController
		(
			const IDragonMotorControllerMap&				motorControllers,
			MotorControllerUsage::MOTOR_CONTROLLER_USAGE	usage
		);
		std::shared_ptr<DragonSolenoid> GetSolenoid
		(
			const DragonSolenoidMap&						solenoids,
			SolenoidUsage::SOLENOID_USAGE					usage
		);
		DragonServo* GetServo
		(
			const ServoMap&									servos,
			ServoUsage::SERVO_USAGE							usage
		);
		std::shared_ptr<DragonDigitalInput> GetDigitalInput
		(
			const DigitalInputMap&							digitaInputs,
			DigitalInputUsage::DIGITAL_SENSOR_USAGE			usage
		);

		DragonAnalogInput* GetAnalogInput
		(
			const AnalogInputMap&							analogInputs,
			DragonAnalogInput::ANALOG_SENSOR_TYPE			usage
		);

		MechanismFactory();
		virtual ~MechanismFactory() = default;

		static MechanismFactory*	m_mechanismFactory;
		

		Intake* 		m_leftIntake;
		Intake* 		m_rightIntake;
		Shooter* 		m_shooter;
		Climber*		m_climber;
		Indexer* 		m_indexer;
		Lift* 			m_lift;
		

};