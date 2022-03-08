
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
#include <states/intake/IntakeStateMgr.h>
#include <states/StateStruc.h>



// Third Party Includes

class RightIntakeStateMgr : public IntakeStateMgr
{
    public:
        
		/// @brief  Find or create the state manmanager
		/// @return RightIntakeStateMgr* pointer to the state manager
		static RightIntakeStateMgr* GetInstance();
        void CheckForStateTransition() override;

    protected:
        const StateStruc  m_rightIntakeOffState = {INTAKE_STATE::OFF, StateType::RIGHT_INTAKE, true};
        const StateStruc  m_rightIntakeIntakeState = {INTAKE_STATE::INTAKE, StateType::RIGHT_INTAKE, false};
        const StateStruc  m_rightIntakeExpelState = {INTAKE_STATE::EXPEL, StateType::RIGHT_INTAKE, false};

        const std::map<std::string, StateStruc> m_rightIntakeStateMap
        {
            {INTAKE_STATE_OFF_STRING, m_rightIntakeOffState},
            {INTAKE_STATE_INTAKE_STRING, m_rightIntakeIntakeState},
            {INTAKE_STATE_EXPEL_STRING, m_rightIntakeExpelState}
        };


    private:

        RightIntakeStateMgr();
        ~RightIntakeStateMgr() = default;

		static RightIntakeStateMgr*	m_instance;
};
