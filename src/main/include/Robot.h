  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/motorcontrol/MotorControllerGroup.h>

// phoenix TalonFX 
#include <ctre/Phoenix.h>

// limelight-related libaries

#include <frc/smartdashboard/Smartdashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>
#include <wpi/span.h>

#include <cstdio>
#include <iostream>
#include <string.h>
 
// Camera
#include <cameraserver/CameraServer.h>
//orchestra (literally just music)
#include <ctre/phoenix/music/Orchestra.h>
#include <frc/Filesystem.h>
class Robot : public frc::TimedRobot {
 public:
  frc::Spark m_left_front_Motor{1};
  frc::Spark m_left_rear_Motor{0};
  frc::Spark m_right_front_Motor{3};
  frc::Spark m_right_rear_Motor{2};
  frc::MotorControllerGroup m_right{m_left_front_Motor, m_left_rear_Motor};
  frc::MotorControllerGroup m_left{m_right_front_Motor, m_right_rear_Motor};
  frc::DifferentialDrive m_robotDrive{m_left, m_right};
  frc::XboxController m_driverController{0};

  
  
  TalonFX fx{0};
  Orchestra orc;
  


  //limelight
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  // Target Offset Angle Horizontal
  double tx;
  // Target Offset Angle Vertical
  double ty;
  // Target Area
  double ta;
  // Target Skew
  double ts;
  double steeringadjust;

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

  void SimulationInit() override;
  void SimulationPeriodic() override;

  double AutoTargetTurn();
  double DetermineDistance();
};
