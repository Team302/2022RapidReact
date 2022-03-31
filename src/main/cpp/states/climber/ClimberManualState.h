
//====================================================================================================================================================
/// Copyright 2022 Lake Orion Robotics FIRST Team 302 
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#pragma once
#include <memory>

#include <controllers/ControlData.h>
#include <gamepad/TeleopControl.h>
#include <hw/interfaces/IDragonMotorController.h>
#include <subsys/Climber.h>
#include <states/IState.h>

#include <units/velocity.h>

class ClimberManualState : public IState
{
    public:

        ClimberManualState
        (
            ControlData*                    controlData,
            ControlData*                    controlData2,
            double                          maxRotationsUpDown,
            double                          maxRotationsRotate
        );
        ClimberManualState() = delete;
        ~ClimberManualState() = default;

        void Init() override;
        void Run() override;
        bool AtTarget() const override;

    private:
        Climber*                                    m_climber;
        TeleopControl*                              m_controller;
        ControlData*                                m_controlDataUpDown;
        ControlData*                                m_controlDataRotate;
        std::shared_ptr<IDragonMotorController>     m_reach;
        std::shared_ptr<IDragonMotorController>     m_rotate;
        double                                      m_upDownMin;
        double                                      m_upDownMax;
        double                                      m_rotateMin;
        double                                      m_rotateMax;
};
