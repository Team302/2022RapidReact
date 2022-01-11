
#pragma once

#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>


#include <memory>

#include <subsys/interfaces/IChassis.h>
#include <hw/interfaces/IDragonMotorController.h>
#include <hw/usages/IDragonMotorControllerMap.h>
#include <subsys/SwerveModule.h>
#include <subsys/SwerveChassis.h>

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

class ChassisFactory
{

		public:
			enum CHASSIS_TYPE
			{
				UNKNOWN_CHASSIS = -1,
				TANK_CHASSIS,
				MECANUM_CHASSIS,
				SWERVE_CHASSIS,
				MAX_CHASSIS_TYPES
			};
			static ChassisFactory* GetChassisFactory();

			IChassis* GetIChassis();

			//=======================================================================================
			// Method:  		CreateChassis
			// Description:		Create a chassis from the inputs
			// Returns:         Void
			//=======================================================================================
			IChassis* CreateChassis
			(
				CHASSIS_TYPE     			        						type,				// <I> - Chassis Type
				std::string													networkTableName,
				std::string													controlFileName,
				units::length::inch_t										wheelDiameter,		// <I> - Diameter of the wheel
			    units::length::inch_t		        						wheelBase,			// <I> - Front-Back distance between wheel centers
				units::length::inch_t		        						track,				// <I> - Left-Right distance between wheels (same axle)
				units::velocity::meters_per_second_t 						maxVelocity,
				units::radians_per_second_t 								maxAngularSpeed,
				units::acceleration::meters_per_second_squared_t 			maxAcceleration,
				units::angular_acceleration::radians_per_second_squared_t 	maxAngularAcceleration,
				const IDragonMotorControllerMap&    						motors, 		        // <I> - Motor Controllers
				std::shared_ptr<SwerveModule>                               frontLeft, 
				std::shared_ptr<SwerveModule>                               frontRight,
				std::shared_ptr<SwerveModule>                               backLeft, 
				std::shared_ptr<SwerveModule>                               backRight, 
				double                                                      odometryComplianceCoefficient
			);

			//=====================================================================================
			/// Method:         CreateSwerveModule
			/// Description:    Find or create the swerve module
			/// Returns:        SwerveModule *    pointer to the swerve module or nullptr if it 
			///                                         doesn't exist and cannot be created.
			//=====================================================================================
			std::shared_ptr<SwerveModule> CreateSwerveModule
			(
				SwerveModule::ModuleID                            			type,
				const IDragonMotorControllerMap&        					motorControllers,   // <I> - Motor Controllers
				std::shared_ptr<ctre::phoenix::sensors::CANCoder>			turnSensor,
				double                                                      turnP,
				double                                                      turnI,
				double                                                      turnD,
				double                                                      turnF,
				double                                                      turnNominalVal,
				double                                                      turnPeakVal,
				double                                                      turnMaxAcc,
				double                                                      turnCruiseVel
			);
			std::shared_ptr<SwerveModule>	GetLeftFrontSwerveModule() { return m_leftFront; }
			std::shared_ptr<SwerveModule> GetLeftBackSwerveModule() { return m_leftBack; }
			std::shared_ptr<SwerveModule>	GetRightFrontSwerveModule() { return m_rightFront; }
			std::shared_ptr<SwerveModule>	GetRightBackSwerveModule() { return m_rightBack; }

		private:
			std::shared_ptr<IDragonMotorController> GetMotorController
			(
				const IDragonMotorControllerMap&				motorControllers,
				MotorControllerUsage::MOTOR_CONTROLLER_USAGE	usage
			);
			ChassisFactory() = default;
			~ChassisFactory() = default;
			IChassis*        m_chassis;
			std::shared_ptr<SwerveModule>	    		m_leftFront;
			std::shared_ptr<SwerveModule>	    		m_leftBack;
			std::shared_ptr<SwerveModule>	    		m_rightFront;
			std::shared_ptr<SwerveModule>	    		m_rightBack;

			static ChassisFactory*	m_chassisFactory;

};
