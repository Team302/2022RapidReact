
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


// C++ Includes

// FRC includes

// Team 302 includes

// Third Party Includes
#include <string>
#include <frc/GenericHID.h>
#include <gamepad/IDragonGamePad.h>
#include <gamepad/DragonXBox.h>
#include <gamepad/DragonGamePad.h>
#include <gamepad/TeleopControl.h>
#include <frc/DriverStation.h>
#include <utils/Logger.h>

using namespace frc;
using namespace std;

//----------------------------------------------------------------------------------
// Method:      GetInstance
// Description: If there isn't an instance of this class, it will create one.  The
//              single class instance will be returned.
// Returns:     OperatorInterface*  instance of this class
//----------------------------------------------------------------------------------
TeleopControl* TeleopControl::m_instance = nullptr; // initialize the instance variable to nullptr
TeleopControl* TeleopControl::GetInstance()
{
    if ( TeleopControl::m_instance == nullptr )
    {
        TeleopControl::m_instance = new TeleopControl();
    }
    return TeleopControl::m_instance;
}
//----------------------------------------------------------------------------------
// Method:      OperatorInterface <<constructor>>
// Description: This will construct and initialize the object.
//              It maps the functions to the buttons/axis.
//---------------------------------------------------------------------------------
TeleopControl::TeleopControl() : m_axisIDs(),
								 m_buttonIDs(),
								 m_controllerIndex(),
								 m_controllers(),
								 m_count( 0 )
{
	for ( int inx=0; inx<DriverStation::kJoystickPorts; ++inx )
	{
		m_controllers[inx] = nullptr;
		if ( DriverStation::GetJoystickIsXbox( inx ) )
		{
            auto xbox = new DragonXBox( inx );
			m_controllers[inx] = xbox;
		}
		else if ( DriverStation::GetJoystickType( inx ) == GenericHID::kHID1stPerson )
		{
            auto gamepad = new DragonGamepad( inx );
			m_controllers[inx] = gamepad;
		}
	}


    // Initialize the items to not defined
	m_axisIDs.resize(FUNCTION_IDENTIFIER::MAX_FUNCTIONS);
	m_buttonIDs.resize(FUNCTION_IDENTIFIER::MAX_FUNCTIONS);
	m_controllerIndex.resize(FUNCTION_IDENTIFIER::MAX_FUNCTIONS);
    for ( int inx=0; inx<FUNCTION_IDENTIFIER::MAX_FUNCTIONS; ++inx )
    {
        m_axisIDs[inx]    		= IDragonGamePad::UNDEFINED_AXIS;
        m_buttonIDs[inx]  		= IDragonGamePad::UNDEFINED_BUTTON;
        m_controllerIndex[inx]  = -1;
    }
/*
	Driver  NOTES FROM TANAY
	L-Stick Robot Drive (Field Orient) 				SWERVE_DRIVE_DRIVE  << SWERVEDIRVE.CPP  <<ROBOT.CPP
	R-Stick X Axis Robot Rotation 					SWERVE_DRIVE_ROTATE  << SWERVEDIRVE.CPP  <<ROBOT.CPP
	Left Bumper - 	Maintain Target					FINDTARGET  << SWERVEDIRVE.CPP
	Right Bumper - Polar Drive        				DRIVE_POLAR  << SWERVEDIRVE.CPP

	Right Trigger - Climber Manual UP   			CLIMBER_MAN_UP <<ClimberStateMgr
	Left Trigger Climber manual DOWN    			CLIMBER_MAN_DOWN  <<ClimberStateMgr

	Select + R-Stick yAxis Pos Climber Arm Forward  SELECT_CLIMBER_ARM + CLIMBER_MAN_UP <<ClimberStateMgr
	Select + R-Stick yAxis Neg - Climber Arm Back   SELECT_CLIMBER_ARM + CLIMBER_MAN_DOWN <<ClimberStateMgr

	A Button - Automated Climber        			CLIMB_AUTO  <<ClimberStateMgr
	B Button - Rezero Pigion    					REZERO_PIGEON  << SWERVEDIRVE.CPP
*/
    auto ctrlNo = 0;
    if ( m_controllers[ctrlNo] != nullptr && DriverStation::GetJoystickIsXbox(ctrlNo) )
    {
		m_controllerIndex[ SWERVE_DRIVE_DRIVE]			= ctrlNo;
		m_axisIDs[ SWERVE_DRIVE_DRIVE]					= IDragonGamePad::LEFT_JOYSTICK_Y;
		m_controllerIndex[ SWERVE_DRIVE_STEER]			= ctrlNo;
		m_axisIDs[ SWERVE_DRIVE_STEER]					= IDragonGamePad::LEFT_JOYSTICK_X;
		m_controllerIndex[ SWERVE_DRIVE_ROTATE]			= ctrlNo;
		m_axisIDs[ SWERVE_DRIVE_ROTATE]					= IDragonGamePad::RIGHT_JOYSTICK_X;
	
		m_controllerIndex[ DRIVE_TO_SHOOTING_SPOT ]		= ctrlNo;
		m_buttonIDs[ DRIVE_TO_SHOOTING_SPOT ]			= IDragonGamePad::A_BUTTON;
		m_controllerIndex[ REZERO_PIGEON ]				= ctrlNo;
		m_buttonIDs[ REZERO_PIGEON ]					= IDragonGamePad::B_BUTTON;
		m_controllerIndex[DRIVE_POLAR] 					= ctrlNo;  
		m_buttonIDs[DRIVE_POLAR] 						= IDragonGamePad::RIGHT_BUMPER;		
		m_controllerIndex[FINDTARGET] 					= ctrlNo;  
		m_buttonIDs[FINDTARGET]	 						= IDragonGamePad::LEFT_BUMPER;	

		m_controllerIndex[HOLD_POSITION]				= ctrlNo;
		m_buttonIDs[HOLD_POSITION]						= IDragonGamePad::X_BUTTON;
		
		//TODO needs hook to states 
		//Analog???
		m_controllerIndex[CLIMBER_MAN_UP]				= ctrlNo;  
		m_axisIDs[CLIMBER_MAN_UP]	 					= IDragonGamePad::RIGHT_TRIGGER;
		m_controllerIndex[CLIMBER_MAN_DOWN]				= ctrlNo;  
		m_axisIDs[CLIMBER_MAN_DOWN]	 				    = IDragonGamePad::LEFT_TRIGGER;	
		m_controllerIndex[CLIMBER_MAN_ROTATE]			= ctrlNo;  
		m_axisIDs[CLIMBER_MAN_ROTATE]	 				= IDragonGamePad::RIGHT_JOYSTICK_Y;	
		// used to enable climber and disable what right-stick Y might be mapped to.
		// used with CLIMBER_MAN_UP and CLIMBER_MAN_DOWN
		m_controllerIndex[PREP_MIDBAR_CLIMB]			= ctrlNo;  
		m_buttonIDs[PREP_MIDBAR_CLIMB]	 				= IDragonGamePad::START_BUTTON;
		m_controllerIndex[ENABLE_CLIMBER]				= ctrlNo;
		m_buttonIDs[ENABLE_CLIMBER]						= IDragonGamePad::LEFT_STICK_PRESSED;

		//m_controllerIndex[CLIMB_AUTO]					= ctrlNo;  
		//m_buttonIDs[CLIMB_AUTO]	 						= IDragonGamePad::A_BUTTON;
    }
    else
    {
        Logger::GetLogger()->LogError( string("TeleopControl::TeleopControl"), string("No controller plugged into port 0"));
    }

/*
	Co-Pilot NOTES FROM TANAY
	Right Bumper - Right Intake         	INTAKE_RIGHT  RightIntakeStateMgr 
	Left Bumper - Left Intake           	INTAKE_LEFT   LeftIntakeStateMgr 
	A- Button - Auto Shoot              	AUTO_SHOOT    ShooterStateMgr
	y- Button - Force Shoot             	MANUAL_SHOOT  ShooterStateMgr
	B - Button - Manual Kicker          	MAN_KICKER    ???
	Left Stick yAxis - Shooter hood manual  SHOOTER_HOOD_MAN  ShooterStateMgr 
	start - shooter off						SHOOTER_OFF    ShooterStateMgr  
	Down d-Pad - start shooter mtr	     	SHOOTER_MTR_ON ShooterStateMgr
	Left d-Pad - Left Intake Expell     	EXPEL_LEFT     LeftIntakeStateMgr
	Right d-Pad - Right Intake Expell   	EXPEL_RIGHT    RightIntakeStateMgr 
*/

    ctrlNo = 1;
    if ( m_controllers[ctrlNo] != nullptr && DriverStation::GetJoystickIsXbox(ctrlNo) )
    {

		m_controllerIndex[INTAKE_LEFT] 		= ctrlNo;
		m_buttonIDs[INTAKE_LEFT]			= IDragonGamePad::LEFT_BUMPER;	
		m_controllerIndex[INTAKE_RIGHT] 	= ctrlNo;
		m_buttonIDs[INTAKE_RIGHT]			= IDragonGamePad::RIGHT_BUMPER;	
		m_controllerIndex[INTAKE_RETRACT_LEFT] = ctrlNo;
		m_axisIDs[INTAKE_RETRACT_LEFT]		= IDragonGamePad::LEFT_TRIGGER;	
		m_controllerIndex[INTAKE_RETRACT_RIGHT] = ctrlNo;
		m_axisIDs[INTAKE_RETRACT_RIGHT]		= IDragonGamePad::RIGHT_TRIGGER;	
		m_controllerIndex[EXPEL_LEFT] 		= ctrlNo;
		m_buttonIDs[EXPEL_LEFT] 			= IDragonGamePad::POV_270;	
		m_controllerIndex[EXPEL_RIGHT] 		= ctrlNo;
		m_buttonIDs[EXPEL_RIGHT] 			= IDragonGamePad::POV_90;
		

		//Manual indexers in case ball gets stuck
		m_controllerIndex[MANUAL_INDEX]		= ctrlNo;
		m_buttonIDs[MANUAL_INDEX]			= IDragonGamePad::B_BUTTON;

        //TODO needs hook for States
		m_controllerIndex[AUTO_SHOOT_HIGH] 		= ctrlNo;
		m_buttonIDs[AUTO_SHOOT_HIGH] 			= IDragonGamePad::A_BUTTON;	
		//m_controllerIndex[AUTO_SHOOT_LOW] 		= ctrlNo;
		//m_buttonIDs[AUTO_SHOOT_LOW] 			= IDragonGamePad::X_BUTTON;	
		m_controllerIndex[INTAKE_DISABLE_LIMIT_SWITCHES] 		= ctrlNo;
		m_buttonIDs[INTAKE_DISABLE_LIMIT_SWITCHES] 			= IDragonGamePad::X_BUTTON;

		m_controllerIndex[MANUAL_SHOOT] 		= ctrlNo;  
		m_buttonIDs[MANUAL_SHOOT] 				= IDragonGamePad::Y_BUTTON;	
		m_controllerIndex[MAN_KICKER] 			= ctrlNo;
		m_buttonIDs[MAN_KICKER] 				= IDragonGamePad::B_BUTTON;	
		m_controllerIndex[SHOOTER_MTR_ON] 		= ctrlNo;
 		m_buttonIDs[SHOOTER_MTR_ON] 			= IDragonGamePad::POV_180;	
 		m_controllerIndex[SHOOTER_OFF] 			= ctrlNo;
 		m_buttonIDs[SHOOTER_OFF]				= IDragonGamePad::START_BUTTON;	
 	

	 	m_controllerIndex[SHOOTER_HOOD_MAN]	= ctrlNo;
		m_axisIDs[SHOOTER_HOOD_MAN]			= IDragonGamePad::LEFT_JOYSTICK_Y;

	}
    else if ( m_controllers[ctrlNo] != nullptr )
    {

	}
	else
	{
        Logger::GetLogger()->LogError( string("TeleopControl::TeleopControl"), string("Controller 1 not handled"));
    }

	ctrlNo = 2;
    if ( m_controllers[ctrlNo] != nullptr && DriverStation::GetJoystickIsXbox(ctrlNo) )
    {
		m_controllerIndex[CLIMBER_STATE_STARTING] 		= ctrlNo;
		m_buttonIDs[CLIMBER_STATE_STARTING] 			= IDragonGamePad::A_BUTTON;	
		m_controllerIndex[CLIMBER_STATE_PREP_MID] 		= ctrlNo;
		m_buttonIDs[CLIMBER_STATE_PREP_MID] 			= IDragonGamePad::X_BUTTON;	
		m_controllerIndex[CLIMBER_STATE_MID] 		= ctrlNo;  
		m_buttonIDs[CLIMBER_STATE_MID] 				= IDragonGamePad::Y_BUTTON;	
		m_controllerIndex[CLIMBER_STATE_FRONT_PREP] 			= ctrlNo;
		m_buttonIDs[CLIMBER_STATE_FRONT_PREP] 				= IDragonGamePad::B_BUTTON;	

		m_controllerIndex[CLIMBER_STATE_FRONT_ROTATE_A] 		= ctrlNo;
 		m_buttonIDs[CLIMBER_STATE_FRONT_ROTATE_A] 			= IDragonGamePad::POV_0;	
 		m_controllerIndex[CLIMBER_STATE_FRONT_ROTATE_B] 			= ctrlNo;
 		m_buttonIDs[CLIMBER_STATE_FRONT_ROTATE_B] 			= IDragonGamePad::POV_90;	
 		m_controllerIndex[CLIMBER_STATE_FRONT_ELEVATE] 			= ctrlNo;
 		m_buttonIDs[CLIMBER_STATE_FRONT_ELEVATE] 			= IDragonGamePad::POV_180;	
 		m_controllerIndex[CLIMBER_STATE_FRONT_ROTATE_TO_HOOK] 			= ctrlNo;
 		m_buttonIDs[CLIMBER_STATE_FRONT_ROTATE_TO_HOOK] 			= IDragonGamePad::POV_270;	


		m_controllerIndex[CLIMBER_STATE_FRONT_LIFT_ROBOT] 		= ctrlNo;
		m_buttonIDs[CLIMBER_STATE_FRONT_LIFT_ROBOT]			= IDragonGamePad::LEFT_BUMPER;	
		m_controllerIndex[CLIMBER_STATE_ROTATE_ARM] 	= ctrlNo;
		m_buttonIDs[CLIMBER_STATE_ROTATE_ARM]			= IDragonGamePad::RIGHT_BUMPER;	
	}
    else if ( m_controllers[ctrlNo] != nullptr )
    {
	}
	else
	{
        Logger::GetLogger()->LogError( string("TeleopControl::TeleopControl"), string("Controller 2 not handled"));
    }

    ctrlNo = 3;
    if ( m_controllers[ctrlNo] != nullptr && DriverStation::GetJoystickIsXbox(ctrlNo) )
    {
		m_controllerIndex[CLIMBER_STATE_BACK_PREP] 		= ctrlNo;
		m_buttonIDs[CLIMBER_STATE_BACK_PREP] 			= IDragonGamePad::A_BUTTON;	
		m_controllerIndex[CLIMBER_STATE_BACK_ROTATE_A] 		= ctrlNo;
		m_buttonIDs[CLIMBER_STATE_BACK_ROTATE_A] 			= IDragonGamePad::X_BUTTON;	
		m_controllerIndex[CLIMBER_STATE_BACK_LIFT] 		= ctrlNo;  
		m_buttonIDs[CLIMBER_STATE_BACK_LIFT] 				= IDragonGamePad::Y_BUTTON;	
		m_controllerIndex[CLIMBER_STATE_BACK_REST] 			= ctrlNo;
		m_buttonIDs[CLIMBER_STATE_BACK_REST] 				= IDragonGamePad::B_BUTTON;	

		m_controllerIndex[CLIMBER_STATE_OFF] 		= ctrlNo;
		m_buttonIDs[CLIMBER_STATE_OFF]			= IDragonGamePad::LEFT_BUMPER;	
		m_controllerIndex[CLIMBER_STATE_MANUAL] 	= ctrlNo;
		m_buttonIDs[CLIMBER_STATE_MANUAL]			= IDragonGamePad::RIGHT_BUMPER;		
	}
    else if ( m_controllers[ctrlNo] != nullptr )
    {
	}
	else
	{
		Logger::GetLogger()->LogError( string("TeleopControl::TeleopControl"), string("Controller 3 not handled"));

	}

    ctrlNo = 4;
    if ( m_controllers[ctrlNo] != nullptr && DriverStation::GetJoystickIsXbox(ctrlNo) )
    {
	}
    else if ( m_controllers[ctrlNo] != nullptr )
    {
	}
	else
	{
        Logger::GetLogger()->LogError( string("TeleopControl::TeleopControl"), string("Controller 4 not handled"));
    }

    ctrlNo = 5;
    if ( m_controllers[ctrlNo] != nullptr && DriverStation::GetJoystickIsXbox(ctrlNo) )
    {
	}
    else if ( m_controllers[ctrlNo] != nullptr )
    {
	}
	else
	{
        Logger::GetLogger()->LogError( string("TeleopControl::TeleopControl"), string("Controller 5 not handled"));
    }
}


//------------------------------------------------------------------
// Method:      SetAxisScaleFactor
// Description: Allow the range of values to be set smaller than
//              -1.0 to 1.0.  By providing a scale factor between 0.0
//              and 1.0, the range can be made smaller.  If a value
//              outside the range is provided, then the value will
//              be set to the closest bounding value (e.g. 1.5 will
//              become 1.0)
// Returns:     void
//------------------------------------------------------------------
void TeleopControl::SetAxisScaleFactor
(
    TeleopControl::FUNCTION_IDENTIFIER  	function,      // <I> - function that will update an axis
    double                                  scaleFactor    // <I> - scale factor used to limit the range
)
{
	int ctlIndex = m_controllerIndex[ function];
	IDragonGamePad::AXIS_IDENTIFIER axis = m_axisIDs[ function ];
    if ( ctlIndex > -1 && axis != IDragonGamePad::AXIS_IDENTIFIER::UNDEFINED_AXIS  )
    {
    	if (m_controllers[ ctlIndex ] != nullptr)
    	{
    		m_controllers[ ctlIndex ]->SetAxisScale( axis,scaleFactor);
    	}
    }
}

void TeleopControl::SetDeadBand
(
	TeleopControl::FUNCTION_IDENTIFIER		function,
	IDragonGamePad::AXIS_DEADBAND			deadband    
)
{
	int ctlIndex = m_controllerIndex[ function];
	IDragonGamePad::AXIS_IDENTIFIER axis = m_axisIDs[ function ];
    if ( ctlIndex > -1 && axis != IDragonGamePad::AXIS_IDENTIFIER::UNDEFINED_AXIS  )
    {
    	if (m_controllers[ ctlIndex ] != nullptr)
    	{
    		m_controllers[ ctlIndex ]->SetAxisDeadband( axis,deadband);
    	}
    }}


//------------------------------------------------------------------
// Method:      SetAxisProfile
// Description: Sets the axis profile for the specifed axis
// Returns:     void
//------------------------------------------------------------------
void TeleopControl::SetAxisProfile
(
    TeleopControl::FUNCTION_IDENTIFIER  function,       // <I> - function that will update an axis
    IDragonGamePad::AXIS_PROFILE        profile         // <I> - profile to use
)
{
	int ctlIndex = m_controllerIndex[ function];
	IDragonGamePad::AXIS_IDENTIFIER axis = m_axisIDs[ function ];
    if ( ctlIndex > -1 && axis != IDragonGamePad::AXIS_IDENTIFIER::UNDEFINED_AXIS  )
    {
    	if (m_controllers[ ctlIndex ] != nullptr)
    	{
    		m_controllers[ ctlIndex ]->SetAxisProfile( axis,profile);
    	}
    }
}
 
//------------------------------------------------------------------
// Method:      GetAxisValue
// Description: Reads the joystick axis, removes any deadband (small
//              value) and then scales as requested.
// Returns:     double   -  scaled axis value
//------------------------------------------------------------------
double TeleopControl::GetAxisValue
(
    TeleopControl::FUNCTION_IDENTIFIER  function    // <I> - function that whose axis will be read
) const
{
    double value = 0.0;
	int ctlIndex = m_controllerIndex[ function];
	IDragonGamePad::AXIS_IDENTIFIER axis = m_axisIDs[ function ];
    if ( ctlIndex > -1 && axis != IDragonGamePad::AXIS_IDENTIFIER::UNDEFINED_AXIS  )
    {
    	if (m_controllers[ ctlIndex ] != nullptr)
    	{
    		value = m_controllers[ ctlIndex ]->GetAxisValue( axis );
    	}
    }
    return value;
}

//------------------------------------------------------------------
// Method:      IsButtonPressed
// Description: Reads the button value.  Also allows POV, bumpers,
//              and triggers to be treated as buttons.
// Returns:     bool   -  scaled axis value
//------------------------------------------------------------------
bool TeleopControl::IsButtonPressed
(
    TeleopControl::FUNCTION_IDENTIFIER  function    // <I> - function that whose button will be read
) const
{
    bool isSelected = false;
	int ctlIndex = m_controllerIndex[ function];
	IDragonGamePad::BUTTON_IDENTIFIER btn = m_buttonIDs[ function ];
    if ( ctlIndex > -1 && btn != IDragonGamePad::BUTTON_IDENTIFIER::UNDEFINED_BUTTON  )
    {
    	if (m_controllers[ ctlIndex ] != nullptr)
    	{
    		isSelected = m_controllers[ ctlIndex ]->IsButtonPressed( btn );
    	}
    }
    return isSelected;
}


