// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <Robot.h>

#include <auton/CyclePrimitives.h>
#include <gamepad/TeleopControl.h>
#include <states/chassis/SwerveDrive.h>
#include <states/climber/ClimberStateMgr.h>
#include <states/Intake/LeftIntakeStateMgr.h>
#include <states/Intake/RightIntakeStateMgr.h>
#include <states/shooter/ShooterStateMgr.h>
#include <subsys/BallTransfer.h>
#include <subsys/ChassisFactory.h>
#include <subsys/Climber.h>
#include <subsys/Intake.h>
#include <subsys/interfaces/IChassis.h>
#include <subsys/MechanismFactory.h>
#include <subsys/Shooter.h>
#include <xmlhw/RobotDefn.h>
#include <subsys/Indexer.h>
#include <subsys/Lift.h>
#include <states/indexer/LeftIndexerStateMgr.h>
#include <states/indexer/RightIndexerStateMgr.h>


void Robot::RobotInit() 
{
    // Read the XML file to build the robot 
    auto defn = new RobotDefn();
    defn->ParseXML();

    // Get local copies of the teleop controller and the chassis
    m_controller = TeleopControl::GetInstance();
    auto factory = ChassisFactory::GetChassisFactory();
    m_chassis = factory->GetIChassis();
    m_swerve = (m_chassis != nullptr) ? new SwerveDrive() : nullptr;
        
    auto mechFactory = MechanismFactory::GetMechanismFactory();
    m_leftIntake = mechFactory->GetLeftIntake();
    m_leftIntakeStateMgr = m_leftIntake != nullptr ? LeftIntakeStateMgr::GetInstance() : nullptr;

    m_rightIntake = mechFactory->GetRightIntake();
    m_rightIntakeStateMgr = m_rightIntake != nullptr ? RightIntakeStateMgr::GetInstance() : nullptr;

    m_leftIndexer = mechFactory->GetLeftIndexer();
    m_leftIndexerStateMgr = m_leftIndexer != nullptr ? LeftIndexerStateMgr::GetInstance() : nullptr;

    m_rightIndexer = mechFactory->GetRightIndexer();
    m_rightIndexerStateMgr = m_rightIndexer != nullptr ? RightIndexerStateMgr::GetInstance() : nullptr;
    
    m_ballTransfer = mechFactory->GetBallTransfer();
    m_ballTransferStateMgr = m_ballTransfer != nullptr ? BallTransferStateMgr::GetInstance() : nullptr;

    m_shooter = mechFactory->GetShooter();
    m_shooterStateMgr = m_shooter != nullptr ? ShooterStateMgr::GetInstance() : nullptr;

    m_lift = mechFactory->GetLift();
    m_liftStateMgr = m_lift != nullptr ? LiftStateMgr::GetInstance() : nullptr;
    
    m_climber = mechFactory->GetClimber();
    m_climberStateMgr = m_climber != nullptr ? ClimberStateMgr::GetInstance() : nullptr;

    m_cyclePrims = new CyclePrimitives();
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() 
{
    if (m_chassis != nullptr)
    {
        m_chassis->UpdateOdometry();
    }
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() 
{
    if (m_cyclePrims != nullptr)
    {
        m_cyclePrims->Init();
    }
}

void Robot::AutonomousPeriodic() 
{
    if (m_cyclePrims != nullptr)
    {
        m_cyclePrims->Run();
    }
}

void Robot::TeleopInit() 
{

    if (m_chassis != nullptr && m_controller != nullptr && m_swerve != nullptr)
    {
        m_swerve->Init();
    }
    if (m_leftIntake != nullptr && m_leftIntakeStateMgr != nullptr)
    {
        m_leftIntakeStateMgr->RunCurrentState();
    }
    if (m_rightIntake != nullptr && m_rightIntakeStateMgr != nullptr)
    {
        m_rightIntakeStateMgr->RunCurrentState();
    }
    if (m_ballTransfer != nullptr && m_ballTransferStateMgr != nullptr)
    {
        m_ballTransferStateMgr->RunCurrentState();
    }
    if (m_shooterStateMgr != nullptr && m_shooter != nullptr)
    {
        m_shooterStateMgr->RunCurrentState();
    }
    if (m_climberStateMgr != nullptr && m_climber != nullptr)
    {
        m_climberStateMgr->RunCurrentState();
    }
    if (m_rightIndexer != nullptr && m_rightIndexerStateMgr != nullptr)
    {
        m_rightIndexerStateMgr->RunCurrentState();
    }
    if (m_leftIndexer != nullptr && m_leftIndexerStateMgr != nullptr)
    {
        m_leftIndexerStateMgr->RunCurrentState();
    }
    if (m_lift != nullptr && m_liftStateMgr != nullptr)
    {
        m_liftStateMgr->RunCurrentState();
    }
}

void Robot::TeleopPeriodic() 
{
    if (m_chassis != nullptr && m_controller != nullptr && m_swerve != nullptr)
    {
        m_swerve->Run();
    }

    if (m_leftIntake != nullptr && m_leftIntakeStateMgr != nullptr)
    {
        m_leftIntakeStateMgr->RunCurrentState();
    }
    if (m_rightIntake != nullptr && m_rightIntakeStateMgr != nullptr)
    {
        m_rightIntakeStateMgr->RunCurrentState();
    }
    if (m_ballTransfer != nullptr && m_ballTransferStateMgr != nullptr)
    {
        m_ballTransferStateMgr->RunCurrentState();
    }
    if (m_shooter != nullptr && m_shooterStateMgr != nullptr)
    {
        m_shooterStateMgr->RunCurrentState();
    }
    if (m_climberStateMgr != nullptr && m_climber != nullptr)
    {
        m_climberStateMgr->RunCurrentState();
    }
    if (m_rightIndexer != nullptr && m_rightIndexerStateMgr != nullptr)
    {
        m_rightIndexerStateMgr->RunCurrentState();
    }
    if (m_leftIndexer != nullptr && m_leftIndexerStateMgr != nullptr)
    {
        m_leftIndexerStateMgr->RunCurrentState();
    }
    if (m_lift != nullptr && m_liftStateMgr != nullptr)
    {
        m_liftStateMgr->RunCurrentState();
    }
}

void Robot::DisabledInit() 
{

}

void Robot::DisabledPeriodic() 
{

}

void Robot::TestInit() 
{

}

void Robot::TestPeriodic() 
{

}

#ifndef RUNNING_FRC_TESTS
int main() 
{
    return frc::StartRobot<Robot>();
}
#endif
