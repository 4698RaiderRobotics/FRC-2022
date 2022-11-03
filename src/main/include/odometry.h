
frc::DifferentialDriveOdometry setupOdometry(rev::SparkMaxRelativeEncoder* LeftEncoder, rev::SparkMaxRelativeEncoder* RightEncoder, AHRS* gyro) {
    LeftEncoder->SetPosition(0);
    RightEncoder->SetPosition(0);
    gyro->Reset();
    gyro->GetAngle();
    auto rot = frc::Rotation2d{units::angle::degree_t{gyro->GetAngle()}};
    frc::DifferentialDriveOdometry odemetry{rot};
    return odemetry;
}