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


// Team 302 includes
#include <states/DriverFeedback.h>
#include <hw/Led.h>

DriverFeedback::DriverFeedback()   
{
    m_shooter = ShooterStateMgr::GetInstance();
    m_led = new Led(1);
    m_climber = ClimberStateMgr::GetInstance();
    m_ballTransfer = BallTransferStateMgr::GetInstance();
}

void DriverFeedback::updateLed()
{
    if(m_climber->GetCurrentState() != ClimberStateMgr::OFF)
    {
        m_led->setColor(Led::RED);
    }
    else if(m_shooter->AtTarget())
    {
        m_led->setColor(Led::BLUE);
    }
    else if(m_shooter->GetCurrentState() == ShooterStateMgr::AUTO_SHOOT_HIGH_GOAL_CLOSE)
    {
        m_led->setColor(Led::GREEN);
    }
    else if(m_shooter->GetCurrentState() == ShooterStateMgr::AUTO_SHOOT_HIGH_GOAL_FAR)
    {
        m_led->setColor(Led::PINK);
    }
    else if(m_shooter->GetCurrentState() == ShooterStateMgr::SHOOT_LOW_GOAL)
    {
        m_led->setColor(Led::BLACK);
    }
    else if(m_ballTransfer->GetCurrentState() == BallTransferStateMgr::HOLD){
        
        m_led->setColor(Led::YELLOW);
    }
    else if(m_shooter->GetCurrentState() == ShooterStateMgr::SHOOT_FAR)
    {
        m_led->setColor(Led::BLUE);
    }
}

 


