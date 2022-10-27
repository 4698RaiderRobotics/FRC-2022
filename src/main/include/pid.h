
#include <ctre/Phoenix.h>
void SetupPID(TalonFX* motor) {
//    motor->Config_kF(1, 0.5);
//https://docs.ctre-phoenix.com/en/latest/ch16_ClosedLoop.html#velocity-closed-loop-control-mode
}

void Shoot(double speed, TalonFX* shootermotor) { 
//m_leftShooterMotor.Set(ControlMode{0}, speed);
if(speed!=0) {
    double rpm = shootermotor->GetSelectedSensorVelocity()*0.29296875;    //double calculated_setpoint =  controller.Calculate(rpm, speed);
    double converted_speed = speed*3.4133333333333336;
    shootermotor->Set(ControlMode::Velocity, 800);
    //m_leftShooterMotor.Set(ControlMode::PercentOutput, calculated_setpoint);
}
else {
    shootermotor->Set(ControlMode::PercentOutput, 0);
}


}