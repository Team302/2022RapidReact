#pragma once 

// C++ Includes
#include <memory>

// FRC includes

// Team 302 includes
#include <subsys/Mech1IndMotor.h>

// Third Party Includes

class IDragonMotorController;

class Intake : public Mech1IndMotor
{
    public:

        Intake
        (
            std::shared_ptr<IDragonMotorController> motor1
        );

        Intake() = delete;
        virtual ~Intake() = default;
};