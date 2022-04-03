
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
#include <hw/usages/AnalogInputMap.h>
#include <hw/usages/DigitalInputMap.h>
#include <hw/usages/DragonSolenoidMap.h>
#include <hw/usages/ServoMap.h>
#include <hw/DragonSolenoid.h>
#include <hw/DragonServo.h>
#include <hw/DragonAnalogInput.h>
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

MechanismFactory::MechanismFactory() : 	m_leftIntake(nullptr),
										m_rightIntake(nullptr),
										m_shooter(nullptr),
										m_climber(nullptr),
										m_indexer(nullptr),
										m_lift(nullptr)
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
	const AnalogInputMap&					analogInputs,
	shared_ptr<CANCoder>					canCoder
)
{

	// Create the mechanism
	switch ( type )
	{
		
		case MechanismTypes::MECHANISM_TYPE::LEFT_INTAKE:
		{
			if (m_leftIntake == nullptr)
			{
				auto intakeMotor = GetMotorController( motorControllers, MotorControllerUsage::MOTOR_CONTROLLER_USAGE::INTAKE_SPIN);
				auto extendMotor = GetMotorController( motorControllers, MotorControllerUsage::MOTOR_CONTROLLER_USAGE::INTAKE_EXTEND);
				if (intakeMotor.get() != nullptr && extendMotor.get() != nullptr)
				{
					m_leftIntake = new Intake(MechanismTypes::MECHANISM_TYPE::LEFT_INTAKE,
											  controlFileName, 
											  networkTableName, 
											  intakeMotor, 
											  extendMotor);
				}
				else
				{
					Logger::GetLogger()->LogError( string("MechansimFactory::CreateIMechanism" ), string("Left Intake motor missing in XML"));
				}
			}
			else
			{
				Logger::GetLogger()->LogError( string("MechansimFactory::CreateIMechanism" ), string("Left Intake already exists") );
			}
		}
		break;

		case MechanismTypes::MECHANISM_TYPE::RIGHT_INTAKE:
		{
			if (m_rightIntake == nullptr)
			{
				auto intakeMotor = GetMotorController( motorControllers, MotorControllerUsage::MOTOR_CONTROLLER_USAGE::INTAKE_SPIN);
				auto extendMotor = GetMotorController( motorControllers, MotorControllerUsage::MOTOR_CONTROLLER_USAGE::INTAKE_EXTEND);
				if (intakeMotor.get() != nullptr && extendMotor.get() != nullptr)
				{
					m_rightIntake = new Intake(MechanismTypes::MECHANISM_TYPE::RIGHT_INTAKE,
											   controlFileName, 
											   networkTableName, 
											   intakeMotor, 
											   extendMotor);
				}
				else
				{
					Logger::GetLogger()->LogError( string("MechansimFactory::CreateIMechanism" ), string("Right Intake motor missing in XML"));
				}
			}
			else
			{
				Logger::GetLogger()->LogError( string("MechansimFactory::CreateIMechanism" ), string("Right Intake already exists") );
			}
		}
		break;

		case MechanismTypes::INDEXER:
		{
			if (m_indexer == nullptr)
			{
				auto ballsensor = GetDigitalInput(digitalInputs, DigitalInputUsage::DIGITAL_SENSOR_USAGE::BALL_PRESENT);
				auto leftIndexer = GetMotorController( motorControllers, MotorControllerUsage::MOTOR_CONTROLLER_USAGE::LEFT_INDEXER);
				auto rightIndexer = GetMotorController( motorControllers, MotorControllerUsage::MOTOR_CONTROLLER_USAGE::RIGHT_INDEXER);
				if (leftIndexer.get() != nullptr && rightIndexer.get() != nullptr && ballsensor.get() != nullptr)
				{
					m_indexer = new Indexer(MechanismTypes::MECHANISM_TYPE::INDEXER,
												controlFileName,
												networkTableName,
												leftIndexer,
												rightIndexer,
												ballsensor);
					Logger::GetLogger()->LogError( string("MechanismFactory::CreateIMechansim"), string("Created Indexer mechanism"));
				}
				else
				{
					Logger::GetLogger()->LogError( string("MechansimFactory::CreateIMechanism" ), string("indexer motor missing in XML"));
				}
			}
			else
			{
				Logger::GetLogger()->LogError( string("MechansimFactory::CreateIMechanism" ), string("indexer already exists") );
			}
		}
		break;

		case MechanismTypes::LIFT:
		{
			if (m_lift == nullptr)
			{
				auto liftMotor = GetMotorController( motorControllers, MotorControllerUsage::MOTOR_CONTROLLER_USAGE::LIFT);
				if (liftMotor.get() != nullptr)
				{
					m_lift = new Lift(MechanismTypes::MECHANISM_TYPE::LIFT,
										controlFileName,
										networkTableName,
										liftMotor);
					Logger::GetLogger()->LogError( string("MechanismFactory::CreateIMechansim"), string("Created Lift mechanism"));
				}
				else
				{
					Logger::GetLogger()->LogError( string("MechansimFactory::CreateIMechanism" ), string("Lift motor is missing in XML") );
				}
			}
			else
			{
				Logger::GetLogger()->LogError( string("MechansimFactory::CreateIMechanism" ), string("Lift motor already exists") );
			}
		}
		break;


		case MechanismTypes::MECHANISM_TYPE::SHOOTER:
		{
			if (m_shooter == nullptr)
			{
				auto shooterMotor = GetMotorController(motorControllers, MotorControllerUsage::MOTOR_CONTROLLER_USAGE::SHOOTER);
				auto shooterMotor2 = GetMotorController(motorControllers, MotorControllerUsage::MOTOR_CONTROLLER_USAGE::SHOOTER2);
				if ( shooterMotor.get() != nullptr && shooterMotor2.get() != nullptr)
				{
					m_shooter = new Shooter(controlFileName, networkTableName, shooterMotor, shooterMotor2);
				}
				else
				{
					Logger::GetLogger()->LogError( string("MechansimFactory::CreateIMechanism" ), string("No Shooter motor exists in XML"));
				}
			}
		}
		break;		
		
		case MechanismTypes::MECHANISM_TYPE::CLIMBER :
		{
			if (m_climber == nullptr)
			{
				//auto liftHeight = GetAnalogInput(analogInputs, DragonAnalogInput::ANALOG_SENSOR_TYPE::ELEVATOR_HEIGHT);
				auto armBackSw = GetDigitalInput(digitalInputs, DigitalInputUsage::CLIMBER_BACK);
				auto liftMotor = GetMotorController( motorControllers, MotorControllerUsage::MOTOR_CONTROLLER_USAGE::CLIMBER_LIFT );
				auto rotateMotor = GetMotorController( motorControllers, MotorControllerUsage::MOTOR_CONTROLLER_USAGE::CLIMBER_ROTATE);
				if ( liftMotor.get() != nullptr && rotateMotor.get() != nullptr )
				{
					m_climber = new Climber(liftMotor, rotateMotor, armBackSw);//, liftHeight);
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
	switch (type)
	{
		case MechanismTypes::MECHANISM_TYPE::CLIMBER:
			return GetClimber();
			break;
			
		case MechanismTypes::MECHANISM_TYPE::LEFT_INTAKE:
			return GetLeftIntake();
			break;

		case MechanismTypes::MECHANISM_TYPE::RIGHT_INTAKE:
			return GetRightIntake();
			break;

		case MechanismTypes::MECHANISM_TYPE::INDEXER:
			return GetIndexer();
			break;
	
		case MechanismTypes::MECHANISM_TYPE::LIFT:
			return GetLift();
			break;
	
		case MechanismTypes::MECHANISM_TYPE::SHOOTER:
			return GetShooter();
			break;

		default:
			return nullptr;
			break;
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
	
DragonAnalogInput* MechanismFactory::GetAnalogInput
(
	const AnalogInputMap&							analogInputs,
	DragonAnalogInput::ANALOG_SENSOR_TYPE			usage
)
{
	DragonAnalogInput* anIn = nullptr;
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
	
	if ( anIn == nullptr )
	{
		string msg = "analog input is nullptr; usage = ";
		msg += to_string( usage );
		Logger::GetLogger()->LogError( string( "MechanismFactory::GetAnalogInput" ), msg );
	}
	return anIn;
}



