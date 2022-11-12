
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/Smartdashboard.h>
#include <units/angular_velocity.h>
void SetupPID(TalonFX* motor) {
    //Velocity Closed Loop: kF is multiplied by target velocity and added to output.
    //https://docs.ctre-phoenix.com/en/latest/ch16_ClosedLoop.html#how-to-calculate-kf
    //f-gain: (100% * 1023 / [measured velocity] ) 
    frc::SmartDashboard::SetDefaultNumber("kF", 0.109700);
    frc::SmartDashboard::SetDefaultNumber("kP", 0.220000);

//    motor->Config_kF(1, 0.5);
//https://docs.ctre-phoenix.com/en/latest/ch16_ClosedLoop.html#velocity-closed-loop-control-mode
}
void PIDTuner(TalonFX* shootermotor) {
    double kF = frc::SmartDashboard::GetNumber("kF", 0);
    double kP = frc::SmartDashboard::GetNumber("kP", 0);
    shootermotor->Config_kF(0,kF);
    shootermotor->Config_kP(0,kP);
}

