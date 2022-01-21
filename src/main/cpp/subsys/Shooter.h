// C++ Includes
#include <memory>

// FRC includes

// Team 302 includes
#include <subsys/Mech1IndMotor.h>

// Third Party Includes

class IDragonMotorController;

class Shooter : public Mech1IndMotor
{
    public:

        Shooter
        (
            std::string                             networkTableName,
	        std::string                             controlFileName,
            std::shared_ptr<IDragonMotorController> shooterMotor
        );

        Shooter() = delete;
        virtual ~Shooter() = default;
};