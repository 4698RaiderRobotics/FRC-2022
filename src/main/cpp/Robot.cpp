#include "Robot.h"
#include "odometry.h"
// to-do
// implement https://docs.wpilib.org/en/latest/docs/software/pathplanning/trajectory-tutorial/index.html
void Robot::RobotInit() {
  SetupMotors();
  m_odometry = setupOdometry(&DriveEncoders[0], &DriveEncoders[2], ahrs);
  //camera
  frc::CameraServer::StartAutomaticCapture();
  ResetEncoders();
  SetupPID(&m_leftShooterMotor);
  //frc::SmartDashboard::PutNumber("rpm_target", 6000);
  
  //orc.LoadMusic("amongus.chrp");
  try
  {
    ahrs = new AHRS(frc::SPI::kMXP);
  }
  catch(const std::exception& e)
  {
    std::cerr << "Error Instatiating navX:\n" << e.what() << '\n';
  }
  
}
void Robot::RobotPeriodic() {
  PIDTuner(&m_leftShooterMotor, setpoint);
  tx = table->GetNumber("tx",0.0);
  ty = table->GetNumber("ty",0.0);
  ta = table->GetNumber("ta",0.0);
  ts = table->GetNumber("ts",0.0);
  double display_velocity = m_leftShooterMotor.GetSelectedSensorVelocity()*0.29296875;
  frc::SmartDashboard::PutNumber("wheel_speed", display_velocity);
  get_gyro();
  auto gyro_angle = frc::Rotation2d{units::angle::degree_t{-ahrs->GetAngle()}};
  units::length::meter_t diameter = units::length::inch_t{4};
  units::length::meter_t left{DriveEncoders[0].GetPosition()*diameter*wpi::numbers::pi};
  units::length::meter_t right{DriveEncoders[2].GetPosition()*diameter*wpi::numbers::pi};
  m_odometry.Update(gyro_angle, left, right);
}
void Robot::AutonomousInit() {
  table->PutNumber("pipeline", 0);  
  ResetEncoders();
  //m_leftShooterMotor.Set(ControlMode::PercentOutput, 1);
  //m_leftShooterMotor.Set(ControlMode::PercentOutput, controller.Calculate(m_leftShooterMotor.GetSelectedSensorVelocity()*(60000/2048), 6000));
}
void Robot::AutonomousPeriodic() {
  correction = 0;
/*   if(AutoTargetTurn() != 0) {
    correction = AutoTargetTurn();
  } */
  int rotations = 7;
  if(abs(DriveEncoders[1].GetPosition()) < rotations || abs(DriveEncoders[2].GetPosition()) < rotations) {
    //pretty sure Arcade Drive function rotation and speed are flipped for somereason
    m_robotDrive.ArcadeDrive(0, 0.7);
  }
/*   if(abs(DriveEncoders[1].GetPosition()) < rotations || abs(DriveEncoders[2].GetPosition()) < rotations) {
    //pretty sure Arcade Drive function rotation and speed are flipped for somereason
    m_robotDrive.ArcadeDrive(0.7, correction);
  } */
/*   if(m_leftShooterMotor.GetSelectedSensorVelocity() > 5000) {
    m_backTriggerMotor.Set(ControlMode::PercentOutput, 1.0);
    m_frontTriggerMotor.Set(1.0);
  } */
  get_gyro();

}
void Robot::TeleopInit() {
  frc::SmartDashboard::SetDefaultNumber("backspin", 0.8);
  frc::SmartDashboard::SetDefaultNumber("triggerspeed", 2000);
  SetupMotors();
  //testing ↘️⬇️
  table->PutNumber("pipeline", 0);
  m_leftShooterMotor.Set(ControlMode::PercentOutput, 0);
  m_backTriggerMotor.Set(ControlMode::PercentOutput, 0);
    m_frontTriggerMotor.Set(0);
    ResetEncoders();
  //table->PutNumber("pipeline", 2);

}
void Robot::TeleopPeriodic() {
  Shoot(setpoint, &m_leftShooterMotor, &m_backShooterMotor);
  frc::SmartDashboard::PutNumber("raw_flywheel_speed", m_leftShooterMotor.GetSelectedSensorVelocity());
  //double wheel_velocity = m_leftShooterMotor.GetSelectedSensorVelocity();
  frc::SmartDashboard::PutNumber("Target Distance", DetermineDistance());
  if(m_operatorController.GetXButton()) {
    Intake(0.8);
  }
  else if (m_operatorController.GetPOV() == 270) {
    Intake(-0.8);
  }
  else {
    Intake(0);
  }
  if(m_operatorController.GetRightTriggerAxis() > 0.5) {
    setpoint=frc::SmartDashboard::GetNumber("triggerspeed", 2000)*3.4133333333333336;
  }
  else {setpoint=0;}
  if(m_operatorController.GetLeftTriggerAxis() > 0.5) {
    m_backTriggerMotor.Set(ControlMode::PercentOutput, 1.0);
    m_frontTriggerMotor.Set(1.0);
  }
/*   if(m_operatorController.GetLeftTriggerAxis() > 0.5) {
    m_backTriggerMotor.Set(ControlMode::PercentOutput, 1.0);
    m_frontTriggerMotor.Set(1.0);
  }  */
  if ((m_operatorController.GetLeftTriggerAxis() < 0.5) && !m_operatorController.GetXButton()) {
    m_backTriggerMotor.Set(ControlMode::PercentOutput, 0.0);
  }
  if (m_operatorController.GetLeftTriggerAxis() > 0.5) {m_frontTriggerMotor.Set(1);}
  else if(m_operatorController.GetPOV() == 180) {
    m_frontTriggerMotor.Set(-1);
  }
  else {
    m_frontTriggerMotor.Set(0);
  }
  if(m_operatorController.GetRightBumper()) {m_intakeArm.Set(ControlMode::PercentOutput, 0.2);}
    else if(m_operatorController.GetLeftBumper()) {m_intakeArm.Set(ControlMode::PercentOutput, -0.2);}
      else{m_intakeArm.Set(ControlMode::PercentOutput, 0);}
  if(m_driverController.GetAButtonPressed()) {table->PutNumber("pipeline", 0);}
  if (m_driverController.GetAButton()) {correction = AutoTargetTurn();}
  if (m_driverController.GetAButtonReleased()) {
  //table->PutNumber("pipeline", 2);
  }
  if (m_driverController.GetRightY() > 0.4) {m_rightClimber.Set(ControlMode::PercentOutput, 0.5);}
    else if (m_driverController.GetRightY() < -0.4) {m_rightClimber.Set(ControlMode::PercentOutput, -0.5);}
      else {m_rightClimber.Set(ControlMode::PercentOutput, 0);}
  if (m_driverController.GetPOV() == 0) {m_leftClimber.Set(ControlMode::PercentOutput, 0.5);}
    else if (m_driverController.GetPOV() == 180) {m_leftClimber.Set(ControlMode::PercentOutput, -0.5);}
      else {m_leftClimber.Set(ControlMode::PercentOutput, 0);}
  DriveMethod();


}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}
void Robot::TestInit() {
}
void Robot::TestPeriodic() {
  m_leftShooterMotor.Set(ControlMode::PercentOutput, controller.Calculate(m_leftShooterMotor.GetSelectedSensorVelocity()*(600/2048), 6000));
}
void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}
double Robot::AutoTargetTurn(){  
  float heading_error = -tx;
  steeringadjust = 0.0f;
  frc::SmartDashboard::PutNumber("tx", heading_error);
  if (tx > 1.0) {
    //steeringadjust = (tx * -0.025) - 0.15;
    //steeringadjust = -0.5;
    steeringadjust = kp*heading_error - min_command;

  }
  else if (tx < 1.0) {
    //steeringadjust = (tx * -0.025) + 0.15;
    //steeringadjust = 0.5;
    steeringadjust = kp*heading_error + min_command;

  }
  return steeringadjust;
}
double Robot::DetermineDistance(){
  d = (h_2 - h_1) / tan(theta_1 + theta_2);
  return d;
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
