// C++ Includes
#include <memory>
#include <string>


// FRC includes

// Team 302 includes
#include <subsys/BallTransfer.h>
#include <subsys/Mech1IndMotor.h>
#include <hw/interfaces/IDragonMotorController.h>

// Third Party Includes

using namespace std;

BallTransfer::BallTransfer
(
    shared_ptr<IDragonMotorController> motor1 //Motor controller passed in in from mech factory
) : Mech1IndMotor(MechanismTypes::MECHANISM_TYPE::BALL_TRANSFER,  string("balltransfer.xml"),  string("BallTransferNT"), motor1)
//  ^ Creates a 1 motor mechanism of type "Ball Transfer", states control data and network table name, also pass in motor controller
{
}