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

/// --------------------------------------------------------------------------------------------
/// @class MotorData
/// @brief 
///        Createing a motor data class to get the stall current, free current, etc. of a motor.
/// --------------------------------------------------------------------------------------------
#pragma once
// Third Party Includes

//302 includes
#include <hw/interfaces/IDragonMotorController.h>

class MotorData
{
    public:

        MotorData() = default;
        ~MotorData() = default;

    
    int getStallCurrent(IDragonMotorController::MOTOR_TYPE motorType) const;
    double getFreeCurrent(IDragonMotorController::MOTOR_TYPE motorType) const;
    int getFreeSpeed(IDragonMotorController::MOTOR_TYPE motorType) const;
    int getMaximumPower(IDragonMotorController::MOTOR_TYPE motorType) const;
    double getStallTorque(IDragonMotorController::MOTOR_TYPE motorType) const;


    private:
    
    const int stallCurrentValues[18] = {257, //Falcon 500
                                        166, //NEO Motor
                                        111, //NEO 550 Motor
                                        131, //CIM Motor
                                        89,  //Mini CIM Motor
                                        53,  //BAG Motor
                                        134, //775pro Motor
                                        71,  //AndyMark 9015
                                        10,  //AndyMark NeveRest
                                        18,  //AndyMark RS775-125
                                        122, //AndyMark Redline A
                                        11,  //REV Robotgics HD Hex Motor
                                        97,  //BaneBots RS-775 18V
                                        84,  //BaneBots RS-550
                                        11,  //Modern Robotics 12VDC Motor
                                        21,  //Johnson Electric Gear Motor
                                        9,   //TETRIX MAX TorqueNADO Motor
                                        0};  //No motor


};
