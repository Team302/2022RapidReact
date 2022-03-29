#pragma once 

// C++ Includes
#include <memory>

// FRC includes

// Team 302 includes
#include <subsys/Mech1Servo.h>
#include <hw/DragonServo.h>

// Third Party Includes


class CameraServo : public Mech1Servo
{
    public:

        CameraServo
        (
            DragonServo* servo
        );

        CameraServo() = delete;
        virtual ~CameraServo() = default;
};