
#include <gamepad/TeleopControl.h>
#include <hw/interfaces/IDragonMotorController.h>
#include <states/intake/IntakeStateMgr.h>
#include <subsys/Intake.h>


IntakeStateMgr::IntakeStateMgr() : m_buttonTriggerStateChange(false)
{
    
}

/// @brief  run the current state
/// @return void
void IntakeStateMgr::CheckForStateTransition()
{
    auto intake = GetIntake();
    if (intake != nullptr)
    {
        auto intakePressed = IsIntakePressed();
        auto expelPressed = IsExpelPressed();
        auto retractIntake = IsRetractSelected();

        auto currentState = static_cast<INTAKE_STATE>(GetCurrentState());
        auto targetState = currentState;

        auto controller = TeleopControl::GetInstance();
        auto disableLimitSws = controller != nullptr && controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::INTAKE_DISABLE_LIMIT_SWITCHES);

        auto extendMotor = intake->GetSecondaryMotor();
        if (extendMotor.get() != nullptr)
        {
            extendMotor.get()->EnableDisableLimitSwitches(!disableLimitSws);
        }
        

        if (intakePressed  &&  currentState != INTAKE_STATE::INTAKE)
        {
            targetState = INTAKE_STATE::INTAKE;
            m_buttonTriggerStateChange = true;
        }
        else if (expelPressed && currentState != INTAKE_STATE::EXPEL)
        {
            targetState = INTAKE_STATE::EXPEL;
            m_buttonTriggerStateChange = true;
        } 
        else if (retractIntake)
        {
            targetState = INTAKE_STATE::RETRACT;
            m_buttonTriggerStateChange = true;
        }          
        else if (!intakePressed && 
                 !expelPressed && 
                 !retractIntake && 
                 m_buttonTriggerStateChange &&
                 currentState != INTAKE_STATE::OFF)
        {
            targetState = INTAKE_STATE::OFF;
        }

        if (targetState != currentState)
        {
            SetCurrentState(targetState, true);
        }
    } 
}

bool IntakeStateMgr::IsIntaking() const
{
    return (IsIntakePressed() || 
            static_cast<INTAKE_STATE>(GetCurrentState()) == INTAKE_STATE::INTAKE);
}