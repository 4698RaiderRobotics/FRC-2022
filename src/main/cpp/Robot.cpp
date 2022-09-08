// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {
  // todo set all motors to factory defaults?
  fx.ConfigFactoryDefault();

  m_left.SetInverted(true);
  //camera
  frc::CameraServer::StartAutomaticCapture();


/*   orc.AddInstrument(fx);
  
  orc.LoadMusic("sus2.chrp");  */ 

}
void Robot::RobotPeriodic() {
}

void Robot::AutonomousInit() {
  if(!orc.IsPlaying()) {
    orc.Play();
  }
}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  table->PutNumber("pipeline", 1);
}
void Robot::TeleopPeriodic() {
/*   m_robotDrive.TankDrive(-m_driverController.GetRightY(),
                          -m_driverController.GetLeftY()); */
  //m_robotDrive.ArcadeDrive(-m_driverController.GetRightY(),-m_driverController.GetRightX());
  

  if (m_driverController.GetAButton()) {
    printf("steeringadjust: %f \n", AutoTargetTurn());
    //std::cout << "steering adjust" << AutoTargetTurn(); 
    m_robotDrive.ArcadeDrive(0, AutoTargetTurn());
  }
  else {
    table->PutNumber("pipeline", 1);
    m_robotDrive.ArcadeDrive(-m_driverController.GetRightY(),-m_driverController.GetRightX());
  }
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
  printf("testinit");

}
void Robot::TestPeriodic() {
}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

double Robot::AutoTargetTurn(){
  table->PutNumber("pipeline", 0);
  tx = table->GetNumber("tx",0.0);
  ty = table->GetNumber("ty",0.0);
  frc::SmartDashboard::PutNumber("tx", tx);
  frc::SmartDashboard::PutNumber("ty", ty);
  ta = table->GetNumber("ta",0.0);
  ts = table->GetNumber("ts",0.0);
  //  m_robotDrive.ArcadeDrive();
  printf("tx: %f",tx);
  if (tx > 2) {
    //steeringadjust = (tx * -0.025) - 0.15;
    steeringadjust = -0.5;
  }
  else if (tx < -2) {
    //steeringadjust = (tx * -0.025) + 0.15;
    steeringadjust = 0.5;
  }
  else {
    steeringadjust = 0;
  }
  return steeringadjust;
}
double Robot::DetermineDistance(){
  
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
