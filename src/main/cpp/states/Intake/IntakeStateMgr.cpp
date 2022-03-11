
#include <states/intake/IntakeStateMgr.h>
#include <subsys/Intake.h>


IntakeStateMgr::IntakeStateMgr() : m_directStateSet(false)
{
    
}

/// @brief  set the current state, initialize it and run it
/// @return void
void IntakeStateMgr::SetCurrentState
(
    int             stateID,
    bool            run
)
{
    m_directStateSet = true;
    StateMgr::SetCurrentState(stateID, run);
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

        if (intakePressed  &&  currentState != INTAKE_STATE::INTAKE)
        {
            targetState = INTAKE_STATE::INTAKE;
            m_directStateSet = false;
        }
        else if (expelPressed && currentState != INTAKE_STATE::EXPEL)
        {
            targetState = INTAKE_STATE::EXPEL;
            m_directStateSet = false;
        } 
        else if (retractIntake)
        {
            targetState = INTAKE_STATE::RETRACT;
            m_directStateSet = false;
        }          
        else if (!intakePressed && 
                 !expelPressed && 
                 !retractIntake && 
                 !m_directStateSet &&
                 currentState != INTAKE_STATE::OFF)
        {
            targetState = INTAKE_STATE::OFF;
        }

        if (targetState != currentState)
        {
            SetCurrentState(targetState, true);
        }
        auto stopped = intake->StopIfFullyExtended();
        if (!stopped)
        {
            intake->StopIfRetracted();
        }
    } 
}