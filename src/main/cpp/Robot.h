// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <gamepad/TeleopControl.h>
#include <subsys/interfaces/IChassis.h>
#include <states/intake/IntakeStateMgr.h>
#include <subsys/Intake.h>
#include <auton/CyclePrimitives.h>
#include <states/shooter/ShooterStateMgr.h>
#include <subsys/Shooter.h>
#include <states/chassis/SwerveDrive.h>
#include <subsys/Shooter.h>
#include <subsys/BallTransfer.h>
#include <states/BallTransfer/BallTransferStateMgr.h>



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
