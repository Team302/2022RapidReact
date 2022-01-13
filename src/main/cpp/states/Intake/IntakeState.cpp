#include <states/Intake/IntakeState.h>
#include <states/Mech1MotorState.h>
#include <subsys/MechanismFactory.h>
#include <controllers/ControlData.h>

IntakeState::IntakeState
(
    ControlData* control,
    double target
) : Mech1MotorState (MechanismFactory::GetMechanismFactory()->GetIntake(), control, target ) {}