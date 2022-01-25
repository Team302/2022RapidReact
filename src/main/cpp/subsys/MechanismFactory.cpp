
//====================================================================================================================================================
/// Copyright 2022 Lake Orion Robotics FIRST Team 302
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

//========================================================================================================
/// MechanismFactory.cpp
//========================================================================================================
///
/// File Description:
///     This controls the creation of mechanisms/subsystems
///
//========================================================================================================

// C++ Includes
#include <map>
#include <memory>

// FRC includes

// Team 302 includes
#include <hw/interfaces/IDragonMotorController.h>
#include <hw/usages/IDragonMotorControllerMap.h>
//#include <hw/usages/AnalogInputMap.h>
#include <hw/usages/DigitalInputMap.h>
#include <hw/usages/DragonSolenoidMap.h>
#include <hw/usages/ServoMap.h>
#include <hw/DragonSolenoid.h>
#include <hw/DragonServo.h>
//#include <hw/DragonAnalogInput.h>
#include <hw/DragonDigitalInput.h>

#include <subsys/MechanismFactory.h>
#include <subsys/interfaces/IMech.h>
#include <subsys/MechanismTypes.h>
#include <utils/Logger.h>

// Third Party Includes
#include <ctre/phoenix/sensors/CANCoder.h>


using namespace std;
using namespace ctre::phoenix::sensors;


//=====================================================================================
/// Method:         GetMechanismFactory
/// Description:    Find or create the mechanism factory
/// Returns:        MechanismFactory* pointer to the mechanism factory
//=====================================================================================
MechanismFactory* MechanismFactory::m_mechanismFactory = nullptr;
MechanismFactory* MechanismFactory::GetMechanismFactory()
{
	if ( MechanismFactory::m_mechanismFactory == nullptr )
	{
		MechanismFactory::m_mechanismFactory = new MechanismFactory();
	}
	return MechanismFactory::m_mechanismFactory;
}

MechanismFactory::MechanismFactory()
{
}

/// @brief      create the requested mechanism
/// @param [in] MechanismTypes::MECHANISM_TYPE  type - the type of mechanism to create
/// @param [in] const IDragonMotorControllerMap& map of the motor usage to the motor controller
/// @param [in] 
/// @param [in] 
/// @param [in] 
/// @param [in] 
void MechanismFactory::CreateIMechanism
(
	MechanismTypes::MECHANISM_TYPE			type,
	string									networkTableName,
	string									controlFileName,
	const IDragonMotorControllerMap&        motorControllers,   // <I> - Motor Controllers
	const DragonSolenoidMap&                solenoids,
	const ServoMap&						    servos,
	const DigitalInputMap&					digitalInputs,
	shared_ptr<CANCoder>					canCoder
)
{

	// Create the mechanism
	switch ( type )
	{
		/**
		case MechanismTypes::MECHANISM_TYPE::INTAKE:
		{
			if (m_intake == nullptr)
			{
				auto motor = GetMotorController( motorControllers, MotorControllerUsage::MOTOR_CONTROLLER_USAGE::INTAKE );
				if ( motor.get() != nullptr )
				{
					m_intake = new Intake(networkTableName, controlFileName, motor);
				}
				else
				{
					Logger::GetLogger()->LogError( string("MechansimFactory::CreateIMechanism" ), string("No intake motor exists in XML"));
				}
			}
			else
			{
				Logger::GetLogger()->LogError( string("MechansimFactory::CreateIMechanism" ), string("Intake already exists") );
			}
		}
		break;
		**/

		
		case MechanismTypes::MECHANISM_TYPE::CLIMBER :
		{
			if (m_climber == nullptr)
			{
				auto motor1 = GetMotorController( motorControllers, MotorControllerUsage::MOTOR_CONTROLLER_USAGE::CLIMBER_CLIMB );
				auto motor2 = GetMotorController( motorControllers, MotorControllerUsage::MOTOR_CONTROLLER_USAGE::CLIMBER_ROTATE);
				if ( motor1.get() != nullptr && motor2.get() != nullptr )
				{
					m_climber = new Climber(motor1, motor2);
					//m_climber = make_shared<Climber>(motor1, motor2);
				}
				else
				{
					Logger::GetLogger()->LogError( string("MechansimFactory::CreateIMechanism" ), string("No climber motors exist in XML"));
				}
			}
			else
			{
				Logger::GetLogger()->LogError( string("MechansimFactory::CreateIMechanism" ), string("Climber already exists") );
			}
		}
		break;

		default:
		{
			string msg = "unknown Mechanism type ";
			msg += to_string( type );
			Logger::GetLogger()->LogError( "MechanismFactory::CreateIMechanism", msg );
		}
		break;
	}
}

shared_ptr<IDragonMotorController> MechanismFactory::GetMotorController
(
	const IDragonMotorControllerMap&				motorControllers,
	MotorControllerUsage::MOTOR_CONTROLLER_USAGE	usage
)
{
	shared_ptr<IDragonMotorController> motor;
	auto it = motorControllers.find( usage );
	if ( it != motorControllers.end() )  // found it
	{
		motor = it->second;
	}
	else
	{
		string msg = "motor not found; usage = ";
		msg += to_string( usage );
		Logger::GetLogger()->LogError( string( "MechanismFactory::GetMotorController" ), msg );
	}
	
	if ( motor.get() == nullptr )
	{
		string msg = "motor is nullptr; usage = ";
		msg += to_string( usage );
		Logger::GetLogger()->LogError( string( "MechanismFactory::GetMotorController" ), msg );
	}
	return motor;
}


IMech* MechanismFactory::GetMechanism
(
	MechanismTypes::MECHANISM_TYPE	type
) const
{
	
	if (type == MechanismTypes::MECHANISM_TYPE::CLIMBER)
	{
		return GetClimber();
	}
	
	return nullptr;
}


shared_ptr<DragonSolenoid> MechanismFactory::GetSolenoid
(
	const DragonSolenoidMap&						solenoids,
	SolenoidUsage::SOLENOID_USAGE					usage
)
{
	shared_ptr<DragonSolenoid> solenoid;
	auto it = solenoids.find( usage );
	if ( it != solenoids.end() )  // found it
	{
		solenoid = it->second;
	}
	else
	{
		string msg = "solenoid not found; usage = ";
		msg += to_string( usage );
		Logger::GetLogger()->LogError( string( "MechanismFactory::GetSolenoid" ), msg );
	}
	
	if ( solenoid.get() == nullptr )
	{
		string msg = "solenoid is nullptr; usage = ";
		msg += to_string( usage );
		Logger::GetLogger()->LogError( string( "MechanismFactory::GetSolenoid" ), msg );
	}
	return solenoid;
}
DragonServo* MechanismFactory::GetServo
(
	const ServoMap&									servos,
	ServoUsage::SERVO_USAGE							usage
)
{
	DragonServo* servo = nullptr;
	auto it = servos.find( usage );
	if ( it != servos.end() )  // found it
	{
		servo = it->second;
	}
	else
	{
		string msg = "servo not found; usage = ";
		msg += to_string( usage );
		Logger::GetLogger()->LogError( string( "MechanismFactory::GetServo" ), msg );
	}
	
	if ( servo == nullptr )
	{
		string msg = "servo is nullptr; usage = ";
		msg += to_string( usage );
		Logger::GetLogger()->LogError( string( "MechanismFactory::GetServo" ), msg );
	}
	return servo;

}
shared_ptr<DragonDigitalInput> MechanismFactory::GetDigitalInput
(
	const DigitalInputMap&							digitaInputs,
	DigitalInputUsage::DIGITAL_SENSOR_USAGE			usage
)
{
	shared_ptr<DragonDigitalInput> dio;
	auto it = digitaInputs.find( usage );
	if ( it != digitaInputs.end() )  // found it
	{
		dio = it->second;
	}
	else
	{
		string msg = "digital input not found; usage = ";
		msg += to_string( usage );
		Logger::GetLogger()->LogError( string( "MechanismFactory::GetDigitalInput" ), msg );
	}
	
	if ( dio.get() == nullptr )
	{
		string msg = "digital input is nullptr; usage = ";
		msg += to_string( usage );
		Logger::GetLogger()->LogError( string( "MechanismFactory::GetDigitalInput" ), msg );
	}
	return dio;
}
/**
shared_ptr<DragonAnalogInput> MechanismFactory::GetAnalogInput
(
	const AnalogInputMap&							analogInputs,
	AnalogInputUsage::ANALOG_SENSOR_USAGE			usage
)
{
	shared_ptr<DragonAnalogInput> anIn;
	auto it = analogInputs.find( usage );
	if ( it != analogInputs.end() )  // found it
	{
		anIn = it->second;
	}
	else
	{
		string msg = "analog input not found; usage = ";
		msg += to_string( usage );
		Logger::GetLogger()->LogError( string( "MechanismFactory::GetAnalogInput" ), msg );
	}
	
	if ( anIn.get() == nullptr )
	{
		string msg = "analog input is nullptr; usage = ";
		msg += to_string( usage );
		Logger::GetLogger()->LogError( string( "MechanismFactory::GetAnalogInput" ), msg );
	}
	return anIn;
}
**/



