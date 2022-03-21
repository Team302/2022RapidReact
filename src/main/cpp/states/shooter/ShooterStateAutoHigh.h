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

#include <array>

#include <hw/DragonLimelight.h>
#include <states/shooter/ShooterState.h>
#include <subsys/Shooter.h>

class ShooterStateAutoHigh : public ShooterState
{
    public:

        ShooterStateAutoHigh() = delete;
        ShooterStateAutoHigh
        (
            ControlData*                    control, 
            ControlData*                    control2,
            double                          primaryTarget,
            double                          secondaryTarget,
            std::array<double,3>            primaryFunctionCoeff,
            std::array<double,3>            secondaryFunctionCoeff
        );
        ~ShooterStateAutoHigh() = default;
        void Init() override;
    
    private:
        DragonLimelight*        m_dragonLimeLight;
        double                  m_shooterTarget;
        double                  m_shooterTarget2;
        std::array<double,3>    m_primaryFunctionCoeff;
        std::array<double,3>    m_secondaryFunctionCoeff;
        
};