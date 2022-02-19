
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
#include <string>
#include <vector>

// FRC includes

// Team 302 includes
#include <auton/PrimitiveEnums.h>
#include <states/intake/IntakeStateMgr.h>
#include <states/shooter/ShooterStateMgr.h>
#include <subsys/interfaces/IChassis.h>

// Third Party Includes


class PrimitiveParams
{
    public:

        PrimitiveParams
        (
                PRIMITIVE_IDENTIFIER                                id,
                float                                               time,
                float                                               distance,
                float                                               xLoc,
                float                                               yLoc,
                IChassis::HEADING_OPTION                            headingOption,
                float                                               heading,
                float                                               startDriveSpeed,
                float                                               endDriveSpeed,
                std::string                                         pathName,
                IntakeStateMgr::INTAKE_STATE                        leftIntakeState,
                IntakeStateMgr::INTAKE_STATE                        rightIntakeState,
                ShooterStateMgr::SHOOTER_STATE                      shooterState
        );//Constructor. Takes in all parameters

        PrimitiveParams() = delete;
        virtual ~PrimitiveParams() = default;//Destructor


        //Some getters
        PRIMITIVE_IDENTIFIER GetID() const {return m_id;};
        float GetTime() const {return m_time;};
        float GetDistance() const {return m_distance;};
        float GetXLocation() const {return m_xLoc;};
        float GetYLocation() const {return m_yLoc;};
        IChassis::HEADING_OPTION GetHeadingOption() const {return m_headingOption;};
        float GetHeading() const {return m_heading;};
        float GetDriveSpeed() const {return m_startDriveSpeed;};
        float GetEndDriveSpeed() const {return m_endDriveSpeed;};
        std::string GetPathName() const {return m_pathName;};
        IntakeStateMgr::INTAKE_STATE GetLeftIntakeState() const {return m_leftIntakeState;};
        IntakeStateMgr::INTAKE_STATE GetRightIntakeState() const {return m_rightIntakeState;};
        ShooterStateMgr::SHOOTER_STATE GetShooterState() const {return m_shooterState;};


        //Setters
        void SetDistance(float distance) {m_distance = distance;};

    private:
        //Primitive Parameters
        PRIMITIVE_IDENTIFIER                                m_id; //Primitive ID
        float                                               m_time;
        float                                               m_distance;
        float                                               m_xLoc;
        float                                               m_yLoc;
        IChassis::HEADING_OPTION                            m_headingOption;
        float                                               m_heading;
        float                                               m_startDriveSpeed;
        float                                               m_endDriveSpeed;
        std::string                                         m_pathName;
        IntakeStateMgr::INTAKE_STATE                        m_leftIntakeState;
        IntakeStateMgr::INTAKE_STATE                        m_rightIntakeState;
        ShooterStateMgr::SHOOTER_STATE                      m_shooterState;

};

typedef std::vector<PrimitiveParams*> PrimitiveParamsVector;



