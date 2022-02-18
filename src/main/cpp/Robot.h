// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>

#include <auton/CyclePrimitives.h>
#include <gamepad/TeleopControl.h>
#include <states/BallTransfer/BallTransferStateMgr.h>
#include <states/chassis/SwerveDrive.h>
#include <states/intake/IntakeStateMgr.h>
#include <states/shooter/ShooterStateMgr.h>
#include <subsys/BallTransfer.h>
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
  Intake*               m_leftIntake;

  IntakeStateMgr*       m_rightIntakeStateMgr;
  Intake*               m_rightIntake;

  BallTransferStateMgr* m_ballTransferStateMgr;
  BallTransfer*         m_ballTransfer;

  ShooterStateMgr*      m_shooterStateMgr;
  Shooter*              m_shooter;
};
