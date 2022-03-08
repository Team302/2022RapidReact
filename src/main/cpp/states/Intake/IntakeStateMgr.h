
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

#pragma once

// C++ Includes
#include <map>
#include <string>

// FRC includes

// Team 302 includes
#include <states/StateMgr.h>
#include <states/StateStruc.h>



// Third Party Includes

class IntakeStateMgr : public StateMgr
{
    public:
        /// @enum the various states of the Intake
        enum INTAKE_STATE
        {
            OFF,
            INTAKE,
            EXPEL,
            MAX_INTAKE_STATES
        };

        // these are the strings in the XML files
        const std::string INTAKE_STATE_OFF_STRING = std::string("INTAKE_OFF");
        const std::string INTAKE_STATE_INTAKE_STRING = std::string("INTAKE_ON");
        const std::string INTAKE_STATE_EXPEL_STRING = std::string("INTAKE_EXPEL");

        const std::map<std::string, INTAKE_STATE> m_intakeStringEnumMap
        {
            {INTAKE_STATE_OFF_STRING, INTAKE_STATE::OFF},
            {INTAKE_STATE_INTAKE_STRING, INTAKE_STATE::INTAKE},
            {INTAKE_STATE_EXPEL_STRING, INTAKE_STATE::EXPEL}
        };

    protected:

        IntakeStateMgr() = default;
        ~IntakeStateMgr() = default;
};
