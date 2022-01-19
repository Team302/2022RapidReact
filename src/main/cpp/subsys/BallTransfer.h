#pragma once 

// C++ Includes
#include <memory>

// FRC includes

// Team 302 includes
#include <subsys/Mech1IndMotor.h>

// Third Party Includes

class IDragonMotorController;

class BallTransfer : public Mech1IndMotor
{
    public:

        BallTransfer
        (
            std::shared_ptr<IDragonMotorController> motor1
        );

        BallTransfer() = delete;
        virtual ~BallTransfer() = default;
};
