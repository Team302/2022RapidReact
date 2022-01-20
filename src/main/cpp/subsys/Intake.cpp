// C++ Includes
#include <memory>
#include <string>

// FRC includes

// Team 302 includes
#include <subsys/Intake.h>
#include <subsys/Mech1IndMotor.h>
#include <hw/interfaces/IDragonMotorController.h>

// Third Party Includes
using namespace std;

Intake::Intake
(
    string                              controlFileName,
    string                              ntName,                        
    shared_ptr<IDragonMotorController> m_motor
) : Mech1IndMotor( MechanismTypes::MECHANISM_TYPE::INTAKE, controlFileName, ntName, m_motor)
{
}

