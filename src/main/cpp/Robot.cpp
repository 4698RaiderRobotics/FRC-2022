#include "Robot.h"
// to-do
// implement https://docs.wpilib.org/en/latest/docs/software/pathplanning/trajectory-tutorial/index.html
void Robot::RobotInit() {
  SetupMotors();
  //camera
  frc::CameraServer::StartAutomaticCapture();
  ResetEncoders();
}
void Robot::RobotPeriodic() {
  tx = table->GetNumber("tx",0.0);
  ty = table->GetNumber("ty",0.0);
  ta = table->GetNumber("ta",0.0);
  ts = table->GetNumber("ts",0.0);
  frc::SmartDashboard::PutNumber("flywheel speed", m_leftShooterMotor.GetSelectedSensorVelocity()* (60000/2048));
  // Selected sensor Velocity -> rpm
  //  units | 1000 ms | 1 rotation | 60 sec |
  //  ms    | 1 sec   | 2048 units | 1 min |
  // * (60,000/2048)

  //display motor encorers on shuffleboard/smartdashboard
/*   for(int i=0; i<4; i++) {
    frc::SmartDashboard::PutNumber(std::to_string(i), DriveEncoders[i].GetPosition());
  } */

  //frc::DriverStation::GetBatteryVoltage();
  frc::SmartDashboard::PutNumber("rough game time", frc::DriverStation::GetMatchTime());
}
void Robot::AutonomousInit() {
  table->PutNumber("pipeline", 0);  
  ResetEncoders();
  //m_leftShooterMotor.Set(ControlMode::PercentOutput, 1);
  m_leftShooterMotor.Set(ControlMode::PercentOutput, controller.Calculate(m_leftShooterMotor.GetSelectedSensorVelocity()*(60000/2048), 6000));
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
}
void Robot::TeleopInit() {
  m_leftShooterMotor.Set(ControlMode::PercentOutput, 0);
  m_backTriggerMotor.Set(ControlMode::PercentOutput, 0);
  m_frontTriggerMotor.Set(0);
  ResetEncoders();
  table->PutNumber("pipeline", 2);

}
void Robot::TeleopPeriodic() {
  frc::SmartDashboard::PutNumber("Target Distance", DetermineDistance());
  
  if(m_operatorController.GetXButton()) {
    Intake(1);
    //m_backTriggerMotor.Set(ControlMode::PercentOutput, 1.0);

  }
  else{
    Intake(0);
  }
  if(m_operatorController.GetRightTriggerAxis() > 0.5) {
    Shoot(1.0); 
  }
  else {
    Shoot(0.0);
  }
  if(m_operatorController.GetLeftTriggerAxis() > 0.5) {
    m_backTriggerMotor.Set(ControlMode::PercentOutput, 1.0);
    m_frontTriggerMotor.Set(1.0);
  } 
  else if ((m_operatorController.GetLeftTriggerAxis() < 0.5) && !m_operatorController.GetXButton()) {
    m_backTriggerMotor.Set(ControlMode::PercentOutput, 0.0);
  }
  if(m_operatorController.GetLeftTriggerAxis() < 0.5) {
    m_frontTriggerMotor.Set(0.0);
  }
  if(m_operatorController.GetRightBumper()) {
    m_intakeArm.Set(ControlMode::PercentOutput, 0.2);
    //m_intakeArm.Set(ControlMode::Position)'
    //64:1
  }
  else if(m_operatorController.GetLeftBumper()) {
    m_intakeArm.Set(ControlMode::PercentOutput, -0.2);
  }
  else{
    m_intakeArm.Set(ControlMode::PercentOutput, 0);
  }
  if(m_driverController.GetAButtonPressed()) {
    table->PutNumber("pipeline", 0);  
  }
  if (m_driverController.GetAButton()) {
    correction = AutoTargetTurn();
  }
  if (m_driverController.GetAButtonReleased()) {
    table->PutNumber("pipeline", 2);
  }
  if (m_driverController.GetRightY() > 0.4) {
    m_rightClimber.Set(ControlMode::PercentOutput, 0.5);
  }
  else if (m_driverController.GetRightY() < -0.4) {
    m_rightClimber.Set(ControlMode::PercentOutput, -0.5);
  }
  else {
    m_rightClimber.Set(ControlMode::PercentOutput, 0);
  }
  if (m_driverController.GetPOV() == 0) {
    m_leftClimber.Set(ControlMode::PercentOutput, 0.5);
  }
  else if (m_driverController.GetPOV() == 180) {
    m_leftClimber.Set(ControlMode::PercentOutput, -0.5);
  }
  else {
    m_leftClimber.Set(ControlMode::PercentOutput, 0);
  }
  DriveMethod();


}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}
void Robot::TestInit() {}
void Robot::TestPeriodic() {}
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
