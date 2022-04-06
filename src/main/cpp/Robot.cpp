// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <Robot.h>
#include <cameraserver/CameraServer.h>

#include <auton/CyclePrimitives.h>
#include <gamepad/TeleopControl.h>
#include <states/chassis/SwerveDrive.h>
#include <states/climber/ClimberStateMgr.h>
#include <states/indexer/IndexerStateMgr.h>
#include <states/Intake/LeftIntakeStateMgr.h>
#include <states/Intake/RightIntakeStateMgr.h>
#include <states/shooter/ShooterStateMgr.h>
#include <subsys/ChassisFactory.h>
#include <subsys/interfaces/IChassis.h>
#include <xmlhw/RobotDefn.h>


void Robot::RobotInit() 
{
    //CameraServer::SetSize(CameraServer::kSize320x240);
    //CameraServer::StartAutomaticCapture();

    // Read the XML file to build the robot 
    auto defn = new RobotDefn();
    defn->ParseXML();

    // Get local copies of the teleop controller and the chassis
    m_controller = TeleopControl::GetInstance();
    auto factory = ChassisFactory::GetChassisFactory();
    m_chassis = factory->GetIChassis();
    m_swerve = (m_chassis != nullptr) ? new SwerveDrive() : nullptr;
        
    m_leftIntakeStateMgr = LeftIntakeStateMgr::GetInstance();
    m_rightIntakeStateMgr = RightIntakeStateMgr::GetInstance();
    m_indexerStateMgr = IndexerStateMgr::GetInstance();
    m_liftStateMgr = LiftStateMgr::GetInstance();
    m_shooterStateMgr = ShooterStateMgr::GetInstance();
    m_climberStateMgr = ClimberStateMgr::GetInstance();

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
    if (m_leftIntakeStateMgr != nullptr)
    {
        m_leftIntakeStateMgr->RunCurrentState();
    }
    if (m_rightIntakeStateMgr != nullptr)
    {
        m_rightIntakeStateMgr->RunCurrentState();
    }
    if (m_shooterStateMgr != nullptr)
    {
        m_shooterStateMgr->SetCurrentState(ShooterStateMgr::SHOOTER_STATE::PREPARE_TO_SHOOT, true);
    }
    if (m_climberStateMgr != nullptr)
    {
        m_climberStateMgr->RunCurrentState();
    //    m_climberStateMgr->SetCurrentState(ClimberStateMgr::CLIMBER_STATE::CLIMB_MID_BAR, true);
    }
    if (m_indexerStateMgr != nullptr)
    {
        m_indexerStateMgr->RunCurrentState();
    }
    if (m_liftStateMgr != nullptr)
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

    if (m_leftIntakeStateMgr != nullptr)
    {
        m_leftIntakeStateMgr->RunCurrentState();
    }
    if (m_rightIntakeStateMgr != nullptr)
    {
        m_rightIntakeStateMgr->RunCurrentState();
    }
    if (m_shooterStateMgr != nullptr)
    {
        m_shooterStateMgr->RunCurrentState();
    }
    if (m_climberStateMgr != nullptr)
    {
        m_climberStateMgr->RunCurrentState();
    }
    if (m_indexerStateMgr != nullptr)
    {
        m_indexerStateMgr->RunCurrentState();
    }
    if (m_liftStateMgr != nullptr)
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
