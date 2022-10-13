#pragma once
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/Smartdashboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/DriverStation.h>


//limelight 💩{
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>
//}
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
#include <AHRS.h>

struct Robot : public frc::TimedRobot {
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
  TalonSRX* seven_seventies[2] = {&m_intakeSpinMotor, &m_backTriggerMotor};

  TalonFX m_rightShooterMotor{15};
  TalonFX m_leftShooterMotor{19};
  TalonFX m_backShooterMotor{17};
  TalonFX* Talons[3] = {&m_rightShooterMotor, &m_leftShooterMotor, &m_backShooterMotor};
  
  TalonFX m_intakeArm{20};

  frc::XboxController m_driverController{0};
  frc::XboxController m_operatorController{1};
  Orchestra orc;
  //Gyro
  AHRS *ahrs;
  //frc2::PIDController pid{kP, kI, kD};


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

  void SetupMotors(){

    for (rev::CANSparkMax* motor : DriveMotors) {
      motor->RestoreFactoryDefaults();
      motor->SetSmartCurrentLimit(70);
      motor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    }
    for (TalonFX* motor : Talons) {
      motor->ConfigFactoryDefault();
      motor->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration{true, 50, 50, 1});
      motor->ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration{true,100,100,1.0});
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
    m_backShooterMotor.Follow(m_leftShooterMotor);
    m_frontTriggerMotor.RestoreFactoryDefaults();
    m_frontTriggerMotor.SetSmartCurrentLimit(30);
    m_leftFollowMotor.Follow(m_leftLeadMotor);
    m_rightFollowMotor.Follow(m_rightLeadMotor);
    

  
  }
  void DriveMethod(){
    //Forza™️ Controls:
    m_driverController.SetRumble(frc::GenericHID::RumbleType::kRightRumble, m_driverController.GetRightTriggerAxis());
    m_driverController.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, m_driverController.GetLeftTriggerAxis());
    m_robotDrive.ArcadeDrive(-m_driverController.GetLeftX()*0.75,m_driverController.GetLeftTriggerAxis()-m_driverController.GetRightTriggerAxis());
  }
  void Intake(double speed) {
    m_intakeSpinMotor.Set(ctre::phoenix::motorcontrol::ControlMode{0}, speed);
  }
  void Shoot(double speed) { 
    m_leftShooterMotor.Set(ControlMode{0}, speed);
  }
  void get_gyro() {
    if(!ahrs) {
      return;
    }
    frc::SmartDashboard::PutBoolean("IMU_Connected", ahrs->IsConnected());
    frc::SmartDashboard::PutNumber("IMU_Yaw", ahrs->GetYaw());
    frc::SmartDashboard::PutNumber("IMU_Pitch", ahrs->GetPitch());
    frc::SmartDashboard::PutNumber("IMU_Roll", ahrs->GetRoll());
    frc::SmartDashboard::PutNumber("IMU_CompassHeading", ahrs->GetCompassHeading());
    frc::SmartDashboard::PutNumber("IMU_Update_Count", ahrs->GetUpdateCount());
    frc::SmartDashboard::PutNumber("IMU_Byte_Count", ahrs->GetByteCount());
    frc::SmartDashboard::PutNumber("IMU_Timestamp", ahrs->GetLastSensorTimestamp());

  /* These functions are compatible w/the WPI Gyro Class */
    frc::SmartDashboard::PutNumber("IMU_TotalYaw", ahrs->GetAngle());
    frc::SmartDashboard::PutNumber("IMU_YawRateDPS", ahrs->GetRate());

    frc::SmartDashboard::PutNumber("IMU_Accel_X", ahrs->GetWorldLinearAccelX());
    frc::SmartDashboard::PutNumber("IMU_Accel_Y", ahrs->GetWorldLinearAccelY());
    frc::SmartDashboard::PutBoolean("IMU_IsMoving", ahrs->IsMoving());
    frc::SmartDashboard::PutNumber("IMU_Temp_C", ahrs->GetTempC());
    frc::SmartDashboard::PutBoolean("IMU_IsCalibrating", ahrs->IsCalibrating());

    frc::SmartDashboard::PutNumber("Velocity_X", ahrs->GetVelocityX());
    frc::SmartDashboard::PutNumber("Velocity_Y", ahrs->GetVelocityY());
    frc::SmartDashboard::PutNumber("Displacement_X", ahrs->GetDisplacementX());
    frc::SmartDashboard::PutNumber("Displacement_Y", ahrs->GetDisplacementY());

  /* Omnimount Yaw Axis Information                                           */
  /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
    AHRS::BoardYawAxis yaw_axis = ahrs->GetBoardYawAxis();
    frc::SmartDashboard::PutString("YawAxisDirection", yaw_axis.up ? "Up" : "Down");
    frc::SmartDashboard::PutNumber("YawAxis", yaw_axis.board_axis);

  /* Sensor Board Information                                                 */
    frc::SmartDashboard::PutString("FirmwareVersion", ahrs->GetFirmwareVersion());

  /* Quaternion Data                                                          */
  /* Quaternions are fascinating, and are the most compact representation of  */
  /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
  /* from the Quaternions.  If interested in motion processing, knowledge of  */
  /* Quaternions is highly recommended.                                       */
    frc::SmartDashboard::PutNumber("QuaternionW", ahrs->GetQuaternionW());
    frc::SmartDashboard::PutNumber("QuaternionX", ahrs->GetQuaternionX());
    frc::SmartDashboard::PutNumber("QuaternionY", ahrs->GetQuaternionY());
    frc::SmartDashboard::PutNumber("QuaternionZ", ahrs->GetQuaternionZ());
  }
  double AutoTargetTurn();
  double DetermineDistance();
  frc::Field2d m_fieldSim;

};
