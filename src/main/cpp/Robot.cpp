#include "Robot.h"
// to-do
// implement https://docs.wpilib.org/en/latest/docs/software/pathplanning/trajectory-tutorial/index.html
void Robot::RobotInit() {
  SetupMotors();
  //camera
  frc::CameraServer::StartAutomaticCapture();}
void Robot::RobotPeriodic() {
  tx = table->GetNumber("tx",0.0);
  ty = table->GetNumber("ty",0.0);
  ta = table->GetNumber("ta",0.0);
  ts = table->GetNumber("ts",0.0);
  //display motor encorers on shuffleboard/smartdashboard
  for(int i=0; i<4; i++) {
    frc::SmartDashboard::PutNumber(std::to_string(i), DriveEncoders[i].GetPosition());
  }

}
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}
void Robot::TeleopInit() {
  //reset intake arm encoder postition
  m_intakeArm.SetSelectedSensorPosition(0); 
  frc::SmartDashboard::PutBoolean("intake_arm_retracted", true);
}
void Robot::TeleopPeriodic() {
  frc::SmartDashboard::PutNumber("Intake Arm Selected Sensor position", m_intakeArm.GetSelectedSensorPosition());
  if(m_operatorController.GetXButton()) {
    Intake(0.6);
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
    m_backTriggerMotor.Set(ControlMode::PercentOutput, -1.0);
    m_frontTriggerMotor.Set(-1.0);
  } else {
    m_backTriggerMotor.Set(ControlMode::PercentOutput, 0.0);
    m_frontTriggerMotor.Set(0.0);
  }
  if(m_operatorController.GetRightBumper()) {
    double velocity = m_intakeArm.GetSelectedSensorVelocity();
    frc::SmartDashboard::PutNumber("Arm Velocity", velocity);
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
  if (m_driverController.GetAButton()) {
    AutoTargetTurn();
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
  table->PutNumber("pipeline", 0);
  
  float heading_error = -tx;
  steeringadjust = 0.0f;
  printf("tx: %f",tx);
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
