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
  tx = table->GetNumber("tx",0.0);
  ty = table->GetNumber("ty",0.0);
  ta = table->GetNumber("ta",0.0);
  ts = table->GetNumber("ts",0.0);
}

void Robot::AutonomousInit() {
  if(!orc.IsPlaying()) {
    orc.Play();
  }
}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  //table->PutNumber("pipeline", 1);
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
    //table->PutNumber("pipeline", 1);
    //m_robotDrive.ArcadeDrive(-m_driverController.GetRightY(),-m_driverController.GetRightX());
    
    double scaled_inputs = (L/(1+std::pow(wpi::math::e,K*m_driverController.GetRightX())))-1;
    //std::cout << "Input Raw: " << m_driverController.GetRightX() << " Input Scaled " << scaled_inputs << "\n";
    
    m_robotDrive.ArcadeDrive(-m_driverController.GetRightY(),scaled_inputs, false);

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
