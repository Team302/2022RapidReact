
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
#include <string>

// FRC includes
#include <frc/Timer.h>

// Team 302 includes
#include <subsys/interfaces/IMech1IndMotorSolenoid.h>
#include <subsys/MechanismTypes.h>

// Third Party Includes
#include <units/time.h>


// forward declares
class ControlModes;
class IDragonMotorController;
class ControlData;
class DragonSolenoid;
class Mech1IndMotor;
class Mech1Solenoid;

class Mech1IndMotorSolenoid : public IMech1IndMotorSolenoid
{
	public:
        /// @brief Create a generic mechanism wiht 1 independent motor 
        /// @param [in] MechanismTypes::MECHANISM_TYPE the type of mechansim
        /// @param [in] std::string the name of the file that will set control parameters for this mechanism
        /// @param [in] std::string the name of the network table for logging information
        /// @param [in] std::shared_ptr<IDragonMotorController> motor controller used by this mechanism
        Mech1IndMotorSolenoid
        (
            MechanismTypes::MECHANISM_TYPE              type,
            std::string                                 controlFileName,
            std::string                                 networkTableName,
            std::shared_ptr<IDragonMotorController>     motorController,
            std::shared_ptr<DragonSolenoid>             solenoid

        );
	    Mech1IndMotorSolenoid() = delete;
	    ~Mech1IndMotorSolenoid() override;

        /// @brief          Indicates the type of mechanism this is
        /// @return         MechanismTypes::MECHANISM_TYPE
        MechanismTypes::MECHANISM_TYPE GetType() const override;

        /// @brief indicate the file used to get the control parameters from
        /// @return std::string the name of the file 
        std::string GetControlFileName() const override;

        /// @brief indicate the Network Table name used to setting tracking parameters
        /// @return std::string the name of the network table 
        std::string GetNetworkTableName() const override;


        /// @brief log data to the network table if it is activated and time period has past
        void LogData() override;

        /// @brief update the output to the mechanism using the current controller and target value(s)
        /// @return void 
        void Update() override;

        void UpdateTarget
        (
            double      target
        ) override;

        /// @brief  Return the current position of the mechanism.  The value is in inches or degrees.
        /// @return double	position in inches (translating mechanisms) or degrees (rotating mechanisms)
        double GetPosition() const override;

        /// @brief  Get the current speed of the mechanism.  The value is in inches per second or degrees per second.
        /// @return double	speed in inches/second (translating mechanisms) or degrees/second (rotating mechanisms)
        double GetSpeed() const override;

        /// @brief  Set the control constants (e.g. PIDF values).
        /// @param [in] ControlData*                                   pid:  the control constants
        /// @return void
        void SetControlConstants
        (
            int                                         slot,
            ControlData*                                pid                 
        ) override;

        /// @brief      Activate/deactivate pneumatic solenoid
        /// @param [in] bool - true == extend, false == retract
        /// @return     void 
        void ActivateSolenoid
        (
            bool     activate
        ) override;

        /// @brief      Check if the pneumatic solenoid is activated
        /// @return     bool - true == extended, false == retracted
        bool IsSolenoidActivated() const override;
        IMech1IndMotor* Get1IndMotorMech() const override;
        IMech1Solenoid* GetSolenoidMech() const override;

    private:
        Mech1IndMotor*              m_motorMech;
        Mech1Solenoid*              m_solenoidMech;
};



