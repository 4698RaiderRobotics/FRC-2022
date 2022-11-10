#pragma once
#include "./pid.h"

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/PS4Controller.h>
#include <frc/drive/DifferentialDrive.h>
#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/Smartdashboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/DriverStation.h>
#include <networktables/NetworkTableValue.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
//}

#include <array>
#include <cstdio>
#include <cmath>
#include <wpi/span.h>
#include <iostream>
#include <string.h>
#include <wpi/numbers>
#include <wpi/math>

#include <cameraserver/CameraServer.h>
#include <cmath>
#include <algorithm>
#include <string.h>
#include <ctre/phoenix/music/Orchestra.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/BangBangController.h>
#include <frc/AddressableLED.h>

// units {
#include <units/angular_velocity.h>
#include <units/voltage.h>
#include <units/length.h>
#include <units/acceleration.h>
#include <ratio>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
//}
//template <class Distance>
struct Robot : public frc::TimedRobot {
  frc::Timer timer = frc::Timer();

  // Drive Motors
  static const int leftLeadDeviceID = 1, leftFollowDeviceID = 2, rightLeadDeviceID = 3, rightFollowDeviceID = 4;
  rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax* DriveMotors[4]= {&m_leftLeadMotor, &m_rightLeadMotor, &m_leftFollowMotor, &m_rightFollowMotor};
  rev::CANSparkMax m_frontTriggerMotor{6, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkMaxRelativeEncoder DriveEncoders[4] = {DriveMotors[0]->GetEncoder(),DriveMotors[1]->GetEncoder(),DriveMotors[2]->GetEncoder(),DriveMotors[3]->GetEncoder()};  
  frc::DifferentialDrive m_robotDrive{m_leftLeadMotor,m_rightLeadMotor};
  // Intake Motors
  //rev::CANSparkMax m_intakeSpinMotor{10, rev::CANSparkMax::MotorType::kBrushless};
  TalonSRX m_intakeSpinMotor{14};
  TalonSRX m_backTriggerMotor{18}; 
  TalonSRX m_intakeWheel{21};
  TalonSRX* seven_seventies[3] = {&m_intakeSpinMotor, &m_backTriggerMotor, &m_intakeWheel};
  TalonFX m_rightShooterMotor{15};
  TalonFX m_leftShooterMotor{19};
  TalonFX m_backShooterMotor{17};
  TalonFX* Talons[3] = {&m_rightShooterMotor, &m_leftShooterMotor, &m_backShooterMotor};
  
  TalonFX m_intakeArm{20};
  TalonFX m_leftClimber{6};
  TalonFX m_rightClimber{7};
  //frc::XboxController m_driverController{0}; 
  frc::PS4Controller m_driverController{0}; 
  frc::XboxController m_operatorController{1};
  frc::BangBangController controller;
  //double setpoint = 0;
  double kF;
  double kP;
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
  double correction;
  float kp = 0.015f;
  float min_command = 0.2f;
 // https://docs.limelightvision.io/en/latest/_images/DistanceEstimation.jpg
  double h_1; // hieght of camera above the floor
  double h_2; // hieght of reflective target
  double theta_1; // fixed angle of the mounted limelight
  double theta_2; // angle of the camera to targ  et (theta_2 = ty)
  double d; // distance to target (lol) (should be returned by DetermineDistance())
  double steeringadjust;
  // Create a new SimpleMotorFeedforward with gains kS, kV, and kA
  // Distance is measured in meters
  //Find kS, kV, kA experimentally with sysID

  using encoder_tic = units::unit<std::ratio<1,2048>, units::angle::turns>;
  using tics_per_100ms = units::compound_unit<encoder_tic, units::inverse<units::time::deciseconds>>;
  using tics_per_100ms_t = units::unit_t<tics_per_100ms>;
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

  void SetupMotors(){ 
    frc::SmartDashboard::SetDefaultNumber("backspin", 1);
    frc::SmartDashboard::SetDefaultNumber("triggerspeed", 3030);
    for (rev::CANSparkMax* motor : DriveMotors) {
      motor->RestoreFactoryDefaults();
      motor->SetSmartCurrentLimit(70);
      motor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    }
    for (TalonFX* motor : Talons) {
      motor->ConfigFactoryDefault();
      motor->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, 30, 40, 1});
      motor->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration{true,80,100,1.0});
      motor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
      //orc.AddInstrument(*motor);
    }
    for (TalonSRX* motor : seven_seventies) {
      motor->ConfigFactoryDefault();
      motor->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true,30,30,0});
      motor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    }
    //m_rightShooterMotor.SetInverted(true);
    m_backShooterMotor.SetInverted(true);
    m_leftShooterMotor.SetInverted(true);
    m_rightShooterMotor.Follow(m_leftShooterMotor);
    SetupPID(&m_leftShooterMotor);
    //we disabled this for tuning [todo fix ]
    //m_backShooterMotor.Follow(m_leftShooterMotor);
    m_intakeSpinMotor.SetInverted(true);
    m_intakeWheel.Follow(m_intakeSpinMotor);
    m_frontTriggerMotor.RestoreFactoryDefaults();
    //m_frontTriggerMotor.SetInverted(true);
    m_frontTriggerMotor.SetSmartCurrentLimit(30);
    m_leftFollowMotor.Follow(m_leftLeadMotor);
    m_rightFollowMotor.Follow(m_rightLeadMotor);
    m_backTriggerMotor.SetInverted(true);
    m_leftClimber.SetNeutralMode(NeutralMode::Brake);
    m_rightClimber.SetNeutralMode(NeutralMode::Brake);
    for(rev::SparkMaxRelativeEncoder encoder:DriveEncoders) {
      //1/7th
      encoder.SetPositionConversionFactor(0.142857);
    }
/*     units::voltage::volt_t ks{0.5};
    auto kV = units::volt_t{1} * units::second_t{1} / units::meter_t{1};
    auto kA = units::volt_t{1} * units::second_t{1} * units::second_t{1} / units::meter_t{1}; 
    frc::SimpleMotorFeedforward<units::meters> feedforward(ks, kV, kA); */

  
  }
  void DriveMethod(){
    //Forza™️ Controls:
    if(m_driverController.GetCrossButton()){
      m_robotDrive.ArcadeDrive(steeringadjust, 0);
    }
    else {
      m_driverController.SetRumble(frc::GenericHID::RumbleType::kRightRumble, m_driverController.GetR2Axis());
      m_driverController.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, m_driverController.GetR2Axis());
      m_robotDrive.ArcadeDrive(-m_driverController.GetLeftX()*0.75,m_driverController.GetL2Axis()-m_driverController.GetR2Axis());
    }
  }
  void Intake(double speed) {
    m_intakeSpinMotor.Set(ctre::phoenix::motorcontrol::ControlMode{0}, speed);
  }

  void ResetEncoders() {
    for (rev::SparkMaxRelativeEncoder encoder: DriveEncoders) {
      encoder.SetPosition(0);
    }
  }
  void Shoot(TalonFX* shootermotor, TalonFX* m_backShooterMotor) { 
//m_leftShooterMotor.Set(ControlMode{0}, speed);

    units::angular_velocity::revolutions_per_minute_t setpoint{frc::SmartDashboard::GetNumber("setpoint_rpm", 0)};
    tics_per_100ms_t motor_setpoint{setpoint};
    shootermotor->Set(ControlMode::Velocity, motor_setpoint.value());
    if(setpoint > units::revolutions_per_minute_t{0}) {
        double backspin = frc::SmartDashboard::GetNumber("backspin", 1);
        m_backShooterMotor->Set(ControlMode::PercentOutput, backspin);    
    }
    else {
        m_backShooterMotor->Set(ControlMode::PercentOutput, 0);
    }
    
    //m_leftShooterMotor.Set(ControlMode::PercentOutput, calculated_setpoint);
}
  template<typename _Function>
  void Bind(bool buttonstate, _Function __F) {
    if(buttonstate) {
      __F();
    }
  }
  double AutoTargetTurn();
  double DetermineDistance();
  frc::Field2d m_fieldSim;

};
