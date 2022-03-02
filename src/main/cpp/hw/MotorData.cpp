
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

#include <hw/MotorData.h>
#include <hw/interfaces/IDragonMotorController.h>

MotorData::MotorData(){}

int MotorData::getStallCurrent(IDragonMotorController::MOTOR_TYPE motorType) const
{
    switch (motorType)
    {
    case IDragonMotorController::FALCON500:
        return stallCurrentValues[IDragonMotorController::FALCON500];
    case IDragonMotorController::NEOMOTOR:
        return stallCurrentValues[IDragonMotorController::NEOMOTOR];
    case IDragonMotorController::NEO500MOTOR:
        return stallCurrentValues[IDragonMotorController::NEO500MOTOR];
    case IDragonMotorController::CIMMOTOR:
        return stallCurrentValues[IDragonMotorController::CIMMOTOR];
    case IDragonMotorController::MINICIMMOTOR:
        return stallCurrentValues[IDragonMotorController::MINICIMMOTOR];
    case IDragonMotorController::BAGMOTOR:
        return stallCurrentValues[IDragonMotorController::BAGMOTOR];
    case IDragonMotorController::PRO775:
        return stallCurrentValues[IDragonMotorController::PRO775];
    case IDragonMotorController::ANDYMARK9015:
        return stallCurrentValues[IDragonMotorController::ANDYMARK9015];
    case IDragonMotorController::ANDYMARKNEVEREST:
        return stallCurrentValues[IDragonMotorController::ANDYMARKNEVEREST];
    case IDragonMotorController::ANDYMARKRS775125:
        return stallCurrentValues[IDragonMotorController::ANDYMARKRS775125];
    case IDragonMotorController::ANDYMARKREDLINEA:
        return stallCurrentValues[IDragonMotorController::ANDYMARKREDLINEA];
    case IDragonMotorController::REVROBOTICSHDHEXMOTOR:
        return stallCurrentValues[IDragonMotorController::REVROBOTICSHDHEXMOTOR];
    case IDragonMotorController::BANEBOTSRS77518V:
        return stallCurrentValues[IDragonMotorController::BANEBOTSRS77518V];
    case IDragonMotorController::BANEBOTSRS550:
        return stallCurrentValues[IDragonMotorController::BANEBOTSRS550];
    case IDragonMotorController::MODERNROBOTICS12VDCMOTOR:
        return stallCurrentValues[IDragonMotorController::MODERNROBOTICS12VDCMOTOR];
    case IDragonMotorController::JOHNSONELECTRICALGEARMOTOR:
        return stallCurrentValues[IDragonMotorController::JOHNSONELECTRICALGEARMOTOR];
    case IDragonMotorController::TETRIXMAXTORQUENADOMOTOR:
        return stallCurrentValues[IDragonMotorController::TETRIXMAXTORQUENADOMOTOR];                        
        
    default:
        return 0;
    }
}
double MotorData::getFreeCurrent(IDragonMotorController::MOTOR_TYPE motorType) const
{

}
int MotorData::getFreeSpeed(IDragonMotorController::MOTOR_TYPE motorType) const
{

}
int MotorData::getMaximumPower(IDragonMotorController::MOTOR_TYPE motorType) const
{

}
double MotorData::getStallTorque(IDragonMotorController::MOTOR_TYPE motorType) const
{

}


