  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>

#include <rev/CANSparkMax.h>

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
#include <cmath>
#include <wpi/math>
// Camera
#include <cameraserver/CameraServer.h>
//orchestra (literally just music)
#include <ctre/phoenix/music/Orchestra.h>
#include <frc/Filesystem.h>

class Robot : public frc::TimedRobot {
  public:
  static const int leftLeadDeviceID = 1, leftFollowDeviceID = 2, rightLeadDeviceID = 3, rightFollowDeviceID = 4;
  rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  frc::DifferentialDrive m_robotDrive{m_leftLeadMotor,m_rightLeadMotor};
  frc::XboxController m_driverController{0};
  
  double L = 2;
  double K = 0.8;
  double x_0 = 0;  

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
  float kp = -0.1f;
  float min_command = 0.5f;
 // https://docs.limelightvision.io/en/latest/_images/DistanceEstimation.jpg
  double h_1; // hieght of camera above the floor
  double h_2; // hieght of reflective target
  double theta_1; // fixed angle of the mounted limelight
  double theta_2; // angle of the camera to targ  et (theta_2 = ty)
  double d; // distance to target (lol) (should be returned by DetermineDistance())
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
