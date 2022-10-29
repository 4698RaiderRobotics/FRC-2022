
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/Smartdashboard.h>

void SetupPID(TalonFX* motor) {
    //Velocity Closed Loop: kF is multiplied by target velocity and added to output.
    //https://docs.ctre-phoenix.com/en/latest/ch16_ClosedLoop.html#how-to-calculate-kf
    //to-do: Run motor at peak velocity and record velocity
    //f-gain: (100% * 1023 / [measured velocity] ) 
    frc::SmartDashboard::SetDefaultNumber("kF", 0.0465);
    frc::SmartDashboard::SetDefaultNumber("kP", 0);

//    motor->Config_kF(1, 0.5);
//https://docs.ctre-phoenix.com/en/latest/ch16_ClosedLoop.html#velocity-closed-loop-control-mode
}
void PIDTuner(TalonFX* shootermotor, double setpoint) {
    frc::SmartDashboard::PutNumber("flywheel velocity", shootermotor->GetSelectedSensorVelocity());
    //frc::SmartDashboard::PutNumber("velocity setpoint", setpoint);
    double kF = frc::SmartDashboard::GetNumber("kF", 0);
    double kP = frc::SmartDashboard::GetNumber("kP", 0);
    shootermotor->Config_kF(0,kF);
    shootermotor->Config_kP(0,kP);
}

void Shoot(double setpoint, TalonFX* shootermotor, TalonFX* m_backShooterMotor) { 
//m_leftShooterMotor.Set(ControlMode{0}, speed);
    frc::SmartDashboard::PutNumber("velocity setpoint", setpoint);
    double rpm = shootermotor->GetSelectedSensorVelocity()*0.29296875;    //double calculated_setpoint =  controller.Calculate(rpm, speed);
    double converted_speed = setpoint*3.4133333333333336;
    shootermotor->Set(ControlMode::Velocity, setpoint);
    if(setpoint > 0) {
        double backspin = frc::SmartDashboard::GetNumber("backspin", 0.8);
        m_backShooterMotor->Set(ControlMode::PercentOutput, backspin);    
    }
    else {
        m_backShooterMotor->Set(ControlMode::PercentOutput, 0);
    }
    
    //m_leftShooterMotor.Set(ControlMode::PercentOutput, calculated_setpoint);
}