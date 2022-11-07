#include "Robot.h"
using namespace units::literals;
// to-do
// implement https://docs.wpilib.org/en/latest/docs/software/pathplanning/trajectory-tutorial/index.html
void Robot::RobotInit() {
  SetupMotors();

  //camera
  cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
  camera.SetResolution(320,240);
  camera.SetFPS(5);

  ResetEncoders();
  SetupPID(&m_leftShooterMotor);
  //frc::SmartDashboard::PutNumber("rpm_target", 6000);
}
void Robot::RobotPeriodic() {
  PIDTuner(&m_leftShooterMotor);
  tx = table->GetNumber("tx",0.0);
  ty = table->GetNumber("ty",0.0);
  ta = table->GetNumber("ta",0.0);
  ts = table->GetNumber("ts",0.0);
  
  tics_per_100ms_t current_velocity{m_leftShooterMotor.GetSelectedSensorVelocity()};
  frc::SmartDashboard::PutNumber("flywheel_velocity_rpm", units::revolutions_per_minute_t{current_velocity}.value());
  double distance = DetermineDistance();
  frc::SmartDashboard::PutNumber("distance", distance);
}
void Robot::AutonomousInit() {
  table->PutNumber("pipeline", 0);  
  ResetEncoders();
}
void Robot::AutonomousPeriodic() {
  correction = 0;
/*   if(AutoTargetTurn() != 0) {
    correction = AutoTargetTurn();
  } */
  int rotations = 5;
  if(abs(DriveEncoders[1].GetPosition()) < rotations || abs(DriveEncoders[2].GetPosition()) < rotations) {
    //pretty sure Arcade Drive function rotation and speed are flipped for somereason
    m_robotDrive.ArcadeDrive(0, 0.7);
  }
}
void Robot::TeleopInit() {
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
  Shoot(&m_leftShooterMotor, &m_backShooterMotor);
  if(m_operatorController.GetXButton()) {
    Intake(0.8);
  }
  else if (m_operatorController.GetPOV() == 270) {
    Intake(-0.8);
  }
  else {
    Intake(0);
  }
  units::revolutions_per_minute_t setpoint{frc::SmartDashboard::GetNumber("triggerspeed", 2000)};
  units::revolutions_per_minute_t current{frc::SmartDashboard::GetNumber("flywheel_velocity_rpm", 0)};
  frc::SmartDashboard::PutBoolean("ready_to_fire", units::math::abs(setpoint - current) - 300_rpm < units::revolutions_per_minute_t{200});
  if(m_operatorController.GetRightTriggerAxis() > 0.5) {
      frc::SmartDashboard::PutNumber("setpoint_rpm", setpoint.value());
    if (units::math::abs(setpoint - current) - 300_rpm < units::revolutions_per_minute_t{200}) {
      m_operatorController.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 1 - units::math::abs(setpoint - current).value()*0.001);
      //m_operatorController.SetRumble(frc::GenericHID::RumbleType::kRightRumble, 1 - units::math::abs(setpoint - current).value()*0.001);
      m_operatorController.SetRumble(frc::GenericHID::RumbleType::kRightRumble, 1);
      
    }
    else {
      m_operatorController.SetRumble(frc::GenericHID::RumbleType::kLeftRumble,0);
      m_operatorController.SetRumble(frc::GenericHID::RumbleType::kRightRumble,0);

    }
  }
  else {
    frc::SmartDashboard::PutNumber("setpoint_rpm", 0);
    if(units::math::abs(setpoint - current) - 300_rpm >= units::revolutions_per_minute_t{200}) {
      m_operatorController.SetRumble(frc::GenericHID::RumbleType::kLeftRumble,0);
      m_operatorController.SetRumble(frc::GenericHID::RumbleType::kRightRumble,0);
    }
  }
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
  if (m_operatorController.GetPOV() == 0) {
    frc::SmartDashboard::PutNumber("setpoint_rpm", -800);
  }
  if(m_operatorController.GetRightBumper()) {m_intakeArm.Set(ControlMode::PercentOutput, 0.2);}
    else if(m_operatorController.GetLeftBumper()) {m_intakeArm.Set(ControlMode::PercentOutput, -0.2);}
      else{m_intakeArm.Set(ControlMode::PercentOutput, 0);}
  if(m_driverController.GetCrossButtonPressed()) {table->PutNumber("pipeline", 0);}
  if (m_driverController.GetCrossButton()) {correction = AutoTargetTurn();}
  if (m_driverController.GetCrossButtonReleased()) {
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
  units::angle::radian_t theta = units::angle::degree_t{ty}; 
  double d = 86/units::math::tan(units::angle::radian_t{0.54}+theta);
  return d;
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
