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

#include <memory>

#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include <subsys/interfaces/IChassis.h>
#include <subsys/ChassisFactory.h>
#include <subsys/ChassisSpeedCalcEnum.h>
#include <subsys/PoseEstimatorEnum.h>
#include <subsys/SwerveChassis.h>
#include <subsys/SwerveModule.h>
#include <hw/interfaces/IDragonMotorController.h>
#include <hw/usages/IDragonMotorControllerMap.h>

#include <utils/Logger.h>

using namespace std;

ChassisFactory* ChassisFactory::m_chassisFactory = nullptr;
ChassisFactory* ChassisFactory::GetChassisFactory()
{
    if ( ChassisFactory::m_chassisFactory == nullptr )
    {
        ChassisFactory::m_chassisFactory = new ChassisFactory();
    }
    return ChassisFactory::m_chassisFactory;
}

IChassis* ChassisFactory::GetIChassis()
{
    return m_chassis;
}

//=======================================================================================
// Method:  		CreateChassis
// Description:		Create a chassis from the inputs
// Returns:         Void
//=======================================================================================
IChassis* ChassisFactory::CreateChassis
(
    ChassisFactory::CHASSIS_TYPE   	                            type,				// <I> - Chassis Type
    string												        networkTableName,
    string												        controlFileName,
    units::length::inch_t										wheelDiameter,		// <I> - Diameter of the wheel
    units::length::inch_t		        						wheelBase,			// <I> - Front-Back distance between wheel centers
    units::length::inch_t		        						track,				// <I> - Left-Right distance between wheels (same axle)
    units::velocity::meters_per_second_t 						maxVelocity,
    units::radians_per_second_t 								maxAngularSpeed,
    units::acceleration::meters_per_second_squared_t 			maxAcceleration,
    units::angular_acceleration::radians_per_second_squared_t 	maxAngularAcceleration,
 	const IDragonMotorControllerMap&                            motors, 		        // <I> - Motor Controllers
    std::shared_ptr<SwerveModule>                               frontLeft, 
    std::shared_ptr<SwerveModule>                               frontRight,
    std::shared_ptr<SwerveModule>                               backLeft, 
    std::shared_ptr<SwerveModule>                               backRight, 
	PoseEstimatorEnum 										    poseEstOption,
    double                                                      odometryComplianceCoefficient
)
{
    switch ( type )
    {
        case ChassisFactory::CHASSIS_TYPE::TANK_CHASSIS:
        {
            /**
            auto leftMotor = GetMotorController(motors, MotorControllerUsage::MOTOR_CONTROLLER_USAGE::DIFFERENTIAL_LEFT_MAIN);
            auto rightMotor = GetMotorController(motors, MotorControllerUsage::MOTOR_CONTROLLER_USAGE::DIFFERENTIAL_RIGHT_MAIN);
            m_chassis = new DifferentialChassis(leftMotor,
                                                rightMotor,
                                                track,
                                                maxVelocity,
                                                maxAngularSpeed,
                                                wheelDiameter,
                                                networkTableName,
                                                controlFileName);
                                                **/

        }
        break;

        case ChassisFactory::CHASSIS_TYPE::MECANUM_CHASSIS:
        {
            // todo plug in mecanum drive
        }
        break;

        case ChassisFactory::CHASSIS_TYPE::SWERVE_CHASSIS:
        {
            m_chassis = new SwerveChassis( frontLeft, 
                                           frontRight, 
                                           backLeft, 
                                           backRight, 
                                           wheelDiameter,
                                           wheelBase, 
                                           track, 
                                           odometryComplianceCoefficient,
                                           maxVelocity, 
                                           maxAngularSpeed, 
                                           maxAcceleration,
                                           maxAngularAcceleration
                                           //poseEstOption, 
                                           //networkTableName,
                                           //controlFileName
                                           );
        }
        break;

        default:
        break;

    }

    return m_chassis;
}
shared_ptr<IDragonMotorController> ChassisFactory::GetMotorController
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
		Logger::GetLogger()->LogError( string( "ChassisFactory::GetMotorController" ), msg );
	}
	
	if ( motor.get() == nullptr )
	{
		string msg = "motor is nullptr; usage = ";
		msg += to_string( usage );
		Logger::GetLogger()->LogError( string( "ChassisFactory::GetMotorController" ), msg );
	}
	return motor;
}


//=====================================================================================
/// Method:         CreateSwerveModule
/// Description:    Find or create the swerve module
/// Returns:        SwerveModule *    pointer to the swerve module or nullptr if it 
///                                         doesn't exist and cannot be created.
//=====================================================================================
std::shared_ptr<SwerveModule> ChassisFactory::CreateSwerveModule
(
    SwerveModule::ModuleID                                      type, 
    const IDragonMotorControllerMap&        				    motorControllers,   // <I> - Motor Controllers
    std::shared_ptr<ctre::phoenix::sensors::CANCoder>		    canCoder,
    double                                                      turnP,
    double                                                      turnI,
    double                                                      turnD,
    double                                                      turnF,
    double                                                      turnNominalVal,
    double                                                      turnPeakVal,
    double                                                      turnMaxAcc,
    double                                                      turnCruiseVel
)
{
    std::shared_ptr<SwerveModule> swerve = nullptr;
	auto driveMotor = GetMotorController(motorControllers, MotorControllerUsage::MOTOR_CONTROLLER_USAGE::SWERVE_DRIVE);
	auto turnMotor  = GetMotorController(motorControllers, MotorControllerUsage::MOTOR_CONTROLLER_USAGE::SWERVE_TURN);

    switch (type)
    {
        case SwerveModule::ModuleID::LEFT_FRONT:
            if ( m_leftFront.get() == nullptr )
            {
                m_leftFront = make_shared<SwerveModule>(type, 
                                                        driveMotor, 
                                                        turnMotor, 
                                                        canCoder, 
                                                        turnP,
                                                        turnI,
                                                        turnD,
                                                        turnF,
                                                        turnNominalVal,
                                                        turnPeakVal,
                                                        turnMaxAcc,
                                                        turnCruiseVel );
            }
            swerve = m_leftFront;
            break;

        case SwerveModule::ModuleID::LEFT_BACK:
            if ( m_leftBack.get() == nullptr )
            {
                m_leftBack = make_shared<SwerveModule>( type, 
                                                        driveMotor, 
                                                        turnMotor, 
                                                        canCoder, 
                                                        turnP,
                                                        turnI,
                                                        turnD,
                                                        turnF,
                                                        turnNominalVal,
                                                        turnPeakVal,
                                                        turnMaxAcc,
                                                        turnCruiseVel );
            }
            swerve = m_leftBack;

            break;

        case SwerveModule::ModuleID::RIGHT_FRONT:
            if ( m_rightFront.get() == nullptr )
            {
                m_rightFront = make_shared<SwerveModule>(type, 
                                                         driveMotor, 
                                                         turnMotor, 
                                                         canCoder, 
                                                         turnP,
                                                         turnI,
                                                         turnD,
                                                         turnF,
                                                         turnNominalVal,
                                                         turnPeakVal,
                                                         turnMaxAcc,
                                                         turnCruiseVel );
           }
            swerve = m_rightFront;
            break;

        case SwerveModule::ModuleID::RIGHT_BACK:
            if ( m_rightBack.get() == nullptr )
            {
                m_rightBack = make_shared<SwerveModule>(type, 
                                                        driveMotor, 
                                                        turnMotor, 
                                                        canCoder, 
                                                        turnP,
                                                        turnI,
                                                        turnD,
                                                        turnF,
                                                        turnNominalVal,
                                                        turnPeakVal,
                                                        turnMaxAcc,
                                                        turnCruiseVel );
            }            
            swerve = m_rightBack;
            break;

        default:
            break;
    }

    return swerve;
}

