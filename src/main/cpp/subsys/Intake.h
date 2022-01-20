#pragma once 

// C++ Includes
#include <memory>
#include <string>

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
            std::string                             controlFileName,
            std::string                             ntName,
            std::shared_ptr<IDragonMotorController> motor1
        );

        Intake() = delete;
        virtual ~Intake() = default;
};