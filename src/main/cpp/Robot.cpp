#include "Robot.h"
// to-do
// implement https://docs.wpilib.org/en/latest/docs/software/pathplanning/trajectory-tutorial/index.html
void Robot::RobotInit() {
  SetupMotors();
  //camera
  frc::CameraServer::StartAutomaticCapture();

}
void Robot::RobotPeriodic() {
  tx = table->GetNumber("tx",0.0);
  ty = table->GetNumber("ty",0.0);
  ta = table->GetNumber("ta",0.0);
  ts = table->GetNumber("ts",0.0);

}
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}
void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  //m_intakeSpinMotor.Set(0.5);
  //m_intakeSpoolMotor.Set(TalonSRXControlMode{0}, -1.0);
  //m_frontSpoolMotor.Set(1);
  //m_rightShooterMotor.Set(ControlMode{0}, 10);
  //m_leftShooterMotor.Set(ControlMode{0}, -10);
  //m_backShooterMotor.Set(ControlMode{0}, -1.0);
  DriveMethod();

}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}
void Robot::TestInit() {}
void Robot::TestPeriodic() {}
void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}
void Robot::DriveMethod(){
  //Forza™️ Controls:
  m_driverController.SetRumble(frc::GenericHID::RumbleType::kRightRumble, m_driverController.GetRightTriggerAxis());
  m_driverController.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, m_driverController.GetLeftTriggerAxis());
  m_robotDrive.ArcadeDrive(-m_driverController.GetRightX()*0.75,m_driverController.GetLeftTriggerAxis()-m_driverController.GetRightTriggerAxis());
}

void Robot::Intake(){

  m_intakeSpinMotor.Set(m_operatorController.GetLeftTriggerAxis());
}
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
