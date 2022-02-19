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

// FRC includes
#include <frc/motorcontrol/Spark.h>
// Team 302 includes
#include <hw/Led.h>

Led::Led
(
    int id
)   
{
     m_spark = new frc::Spark(id);
}  

void Led::setColor(LED_COLORS color) 
{
    switch(color)
    {
        case BLUE:
            m_spark->Set(m_BLUE);
        break;

        case GREEN:
            m_spark->Set(m_GREEN);
        break;

        case PINK:
            m_spark->Set(m_PINK);
        break;

        case BLACK:
            m_spark->Set(m_BLACK);
        break;

        case RED:
            m_spark->Set(m_RED);
        break;

        default:
        //color blue green    
        m_spark->Set(0.79);
    }
}
 


