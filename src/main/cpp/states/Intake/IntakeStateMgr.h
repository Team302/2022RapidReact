
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

// FRC includes

// Team 302 includes
#include <states/StateMgr.h>
#include <states/StateStruc.h>



// Third Party Includes

class Intake;

class IntakeStateMgr : public StateMgr
{
    public:
        /// @enum the various states of the Intake
        enum INTAKE_STATE
        {
            OFF,
            INTAKE,
            EXPEL,
            RETRACT,
            MAX_INTAKE_STATES
        };

        const std::string m_intakeOffXmlString = "INTAKE_OFF";
        const std::string m_intakeIntakeXmlString = "INTAKE_ON";
        const std::string m_intakeExpelXmlString = "INTAKE_EXPEL";
        const std::string m_intakeRetractXmlString = "INTAKE_RETRACT";
        
        const std::map<const std::string, INTAKE_STATE> m_intakeXmlStringToStateEnumMap
        {   {m_intakeOffXmlString, INTAKE_STATE::OFF},
            {m_intakeIntakeXmlString, INTAKE_STATE::INTAKE},
            {m_intakeExpelXmlString, INTAKE_STATE::EXPEL}
        };

        void CheckForStateTransition() override;

        bool IsIntaking() const;

    protected:

        virtual Intake* GetIntake() const = 0;
        virtual bool IsIntakePressed() const = 0;
        virtual bool IsExpelPressed() const = 0;
        virtual bool IsRetractSelected() const = 0;


        IntakeStateMgr();
        ~IntakeStateMgr() = default;

        bool m_buttonTriggerStateChange;
};
