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
#include <algorithm>
#include <memory>

// FRC includes
#include <frc/drive/Vector2d.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>

// Team 302 Includes
#include <states/chassis/SwerveDrive.h>
#include <hw/DragonPigeon.h>
#include <gamepad/IDragonGamePad.h>
#include <gamepad/TeleopControl.h>
#include <states/IState.h>
#include <subsys/ChassisFactory.h>
#include <hw/factories/PigeonFactory.h>
#include <utils/Logger.h>

using namespace std;
using namespace frc;

/// @brief initialize the object and validate the necessary items are not nullptrs
SwerveDrive::SwerveDrive() : IState(),
                             m_chassis(ChassisFactory::GetChassisFactory()->GetSwerveChassis()),
                             m_controller(TeleopControl::GetInstance()),
                             m_usePWLinearProfile(false)
{
    if (m_controller == nullptr)
    {
        Logger::GetLogger()->LogError(string("SwerveDrive::SwerveDrive"), string("TeleopControl is nullptr"));
    }

    if (m_chassis.get() == nullptr)
    {
        Logger::GetLogger()->LogError(string("SwerveDrive::SwerveDrive"), string("Chassis is nullptr"));
    }
}

/// @brief initialize the profiles for the various gamepad inputs
/// @return void
void SwerveDrive::Init()
{
    auto controller = GetController();
    if (controller != nullptr)
    {
        auto profile = (m_usePWLinearProfile) ? IDragonGamePad::AXIS_PROFILE::PIECEWISE_LINEAR : IDragonGamePad::AXIS_PROFILE::CUBED;
        controller->SetAxisProfile(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_DRIVE, profile);
        controller->SetDeadBand(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_DRIVE, IDragonGamePad::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND);
        controller->SetAxisScaleFactor(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_DRIVE, -0.6);

        controller->SetAxisProfile(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_STEER, profile);
        controller->SetDeadBand(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_STEER, IDragonGamePad::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND);
        controller->SetAxisScaleFactor(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_STEER, -0.6);

        controller->SetAxisProfile(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_ROTATE, profile);
        controller->SetDeadBand(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_ROTATE, IDragonGamePad::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND);
        controller->SetAxisScaleFactor(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_ROTATE, 0.5);
    }
}

/// @brief calculate the output for the wheels on the chassis from the throttle and steer components
/// @return void
void SwerveDrive::Run()
{
    auto controller = GetController();
    if (controller != nullptr)
    {
        IChassis::CHASSIS_DRIVE_MODE mode = controller->IsButtonPressed(TeleopControl::DRIVE_POLAR) ? 
                                                    IChassis::CHASSIS_DRIVE_MODE::POLAR_DRIVE : 
                                                    IChassis::CHASSIS_DRIVE_MODE::FIELD_ORIENTED;
        IChassis::HEADING_OPTION headingOpt = IChassis::HEADING_OPTION::MAINTAIN;
        if (controller->IsButtonPressed(TeleopControl::FINDTARGET))
        {
            headingOpt = IChassis::HEADING_OPTION::TOWARD_GOAL;
        }                                       
        else if (controller->IsButtonPressed(TeleopControl::DRIVE_TO_SHOOTING_SPOT))
        {
            headingOpt = IChassis::HEADING_OPTION::TOWARD_GOAL_DRIVE;
        }
        
        if (controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::REZERO_PIGEON))
        {
            auto factory = PigeonFactory::GetFactory();
            auto m_pigeon = factory->GetPigeon(DragonPigeon::PIGEON_USAGE::CENTER_OF_ROBOT);
            m_pigeon->ReZeroPigeon(0, 0);
            m_chassis.get()->ZeroAlignSwerveModules();
            m_chassis.get()->ReZero();
        }

        if (controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::HOLD_POSITION))
        {
            m_chassis.get()->HoldPosition();
        }

        auto drive = controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_DRIVE);
        auto steer = controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_STEER);
        auto rotate = controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::SWERVE_DRIVE_ROTATE);

        m_chassis->Drive(drive, steer, rotate, mode, headingOpt);
    }
}

/// @brief indicates that we are not at our target
/// @return bool
bool SwerveDrive::AtTarget() const
{
    return false;
}