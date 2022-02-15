#pragma once 

// C++ Includes
#include <memory>

// FRC includes

// Team 302 includes
#include <subsys/Mech1Servo.h>
#include <hw/DragonServo.h>

// Third Party Includes


class Servo : public Mech1Servo
{
    public:

        Servo
        (
            DragonServo* servo
        );

        Servo() = delete;
        virtual ~Servo() = default;
};