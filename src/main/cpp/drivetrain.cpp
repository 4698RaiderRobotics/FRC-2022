#include "drivetrain.h"

void Drivetrain::setupDriveMotors() {
    for (rev::CANSparkMax* motor : DriveMotors) {
      motor->RestoreFactoryDefaults();
      motor->SetSmartCurrentLimit(70);
      motor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    }
    m_leftFollowMotor.Follow(m_leftLeadMotor);
    m_rightFollowMotor.Follow(m_rightLeadMotor);
    for(rev::SparkMaxRelativeEncoder encoder:DriveEncoders) {
      //1/7th
      encoder.SetPositionConversionFactor(0.142857);
    }
    ahrs = new AHRS(frc::SPI::kMXP);
    frc::SmartDashboard::PutData("Field", &m_field);

}
void Drivetrain::Drive(frc::PS4Controller* m_driverController) {
    m_driverController->SetRumble(frc::GenericHID::RumbleType::kRightRumble, m_driverController->GetR2Axis());
    m_driverController->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, m_driverController->GetL2Axis());
    m_robotDrive.ArcadeDrive(-m_driverController->GetLeftX()*0.75,m_driverController->GetL2Axis()-m_driverController->GetR2Axis());

}
void Drivetrain::resetEncoders() {
    for (rev::SparkMaxRelativeEncoder encoder: DriveEncoders) {
      encoder.SetPosition(0);
    }
}
void Drivetrain::updateOdometry() {
    auto gyro_angle = frc::Rotation2d{units::angle::degree_t{-ahrs->GetAngle()}};
    units::length::meter_t diameter = units::length::inch_t{4};
    units::length::meter_t left{DriveEncoders[0].GetPosition()*diameter*wpi::numbers::pi};
    units::length::meter_t right{DriveEncoders[2].GetPosition()*diameter*wpi::numbers::pi};
    m_odometry.Update(gyro_angle, left, right);
    m_field.SetRobotPose(m_odometry.GetPose());
}