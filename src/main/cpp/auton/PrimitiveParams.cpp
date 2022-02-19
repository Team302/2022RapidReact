
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

#include <auton/PrimitiveEnums.h>
#include <auton/PrimitiveParams.h>
#include <states/intake/IntakeStateMgr.h>
#include <states/shooter/ShooterStateMgr.h>
#include <subsys/interfaces/IChassis.h>

PrimitiveParams::PrimitiveParams
(
    PRIMITIVE_IDENTIFIER								id,
    float                       						time,
    float                       						distance,
    float                       						xLoc,
    float                       						yLoc,
	IChassis::HEADING_OPTION		    				headingOpt,
    float                       						heading,
    float                       						startDriveSpeed,
    float                       						endDriveSpeed,
	std::string											pathName,
	IntakeStateMgr::INTAKE_STATE                        leftIntakeState,
	IntakeStateMgr::INTAKE_STATE                        rightIntakeState,
	ShooterStateMgr::SHOOTER_STATE                      shooterState
):	//Pass over parameters to class variables
		m_id(id), //Primitive ID
		m_time(time),
		m_distance(distance),
		m_xLoc(xLoc),
		m_yLoc(yLoc),
		m_headingOption(headingOpt),
		m_heading(heading),
		m_startDriveSpeed(startDriveSpeed),
		m_endDriveSpeed(endDriveSpeed),
		m_pathName (pathName),
		m_leftIntakeState(leftIntakeState),
		m_rightIntakeState(rightIntakeState),
		m_shooterState(shooterState)
{
}




