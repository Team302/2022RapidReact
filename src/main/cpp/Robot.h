// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>

#include <auton/CyclePrimitives.h>
#include <gamepad/TeleopControl.h>
#include <states/cameraServo/CameraServoStateMgr.h>
#include <states/chassis/SwerveDrive.h>
#include <states/climber/ClimberStateMgr.h>
#include <states/indexer/IndexerStates.h>
#include <states/intake/IntakeStateMgr.h>
#include <states/lift/LiftStateMgr.h>
#include <states/shooter/ShooterStateMgr.h>
#include <subsys/Intake.h>
#include <subsys/interfaces/IChassis.h>
#include <subsys/Shooter.h>




class Robot : public frc::TimedRobot 
{
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

 private:
  TeleopControl*        m_controller;
  IChassis*             m_chassis;
  CyclePrimitives*      m_cyclePrims;
  frc::Timer*           m_timer;
  SwerveDrive*          m_swerve;

  IntakeStateMgr*       m_leftIntakeStateMgr;
  IntakeStateMgr*       m_rightIntakeStateMgr;
  IndexerStates*        m_leftIndexerStateMgr;
  IndexerStates*        m_rightIndexerStateMgr;
  LiftStateMgr*         m_liftStateMgr;
  ShooterStateMgr*      m_shooterStateMgr;
  ClimberStateMgr*      m_climberStateMgr;

  CameraServoStateMgr*  m_cameraServoStateMgr;
};
