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

//C++ Includes
#include <memory>

//Team 302 Includes
#include <subsys/Mech2IndMotors.h>

class IDragonMotorController;
class DragonDigitalInput;
class DragonAnalogInput;

class Climber : public Mech2IndMotors
{
    public:
        Climber
        (
            std::shared_ptr<IDragonMotorController> reachMotor,
            std::shared_ptr<IDragonMotorController> rotateMotor,
            std::shared_ptr<DragonDigitalInput>     armBackSw//,
            //DragonAnalogInput*                      elevatorHeight
        );

        Climber() = delete;
        virtual ~Climber() = default;

        /// @brief update the output to the mechanism using the current controller and target value(s)
        /// @return void 
        void Update() override;

        /// @brief log data to the network table if it is activated and time period has past
        void LogData() override;

        double GetMinReach() const {return m_reachMin;}
        double GetMaxReach() const {return m_reachMax;}
        double GetMinRotate() const {return m_rotateMin;}
        double GetMaxRotate() const {return m_rotateMax;}

        bool IsLiftStalled() const;
        bool IsRotateStalled() const;

    private:
        static double GetPositionInInches
        (
            std::shared_ptr<IDragonMotorController> motor
        );
        static double GetPositionInDegrees
        (
            std::shared_ptr<IDragonMotorController> motor
        );

        bool IsAtMaxReach
        (
            std::shared_ptr<IDragonMotorController> motor,
            double                                  currentPos
        ) const;
        bool IsAtMinReach
        (
            std::shared_ptr<IDragonMotorController> motor,
            double                                  currentPos
        ) const;
        bool IsAtMaxRotation
        (
            std::shared_ptr<IDragonMotorController> motor,
            double                                  currentPos
        ) const;
        bool IsAtMinRotation
        (
            std::shared_ptr<IDragonMotorController> motor,
            double                                  currentPos
        ) const;

        double                              m_reachMin;
        double                              m_reachMax;
        double                              m_rotateMin;
        double                              m_rotateMax;
        std::shared_ptr<DragonDigitalInput> m_armBack;
        //DragonAnalogInput*                  m_elevatorHeight;
};