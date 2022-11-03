#pragma once
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/PS4Controller.h>
#include <frc/drive/DifferentialDrive.h>
#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/Smartdashboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/DriverStation.h>
#include <AHRS.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/geometry/Rotation2d.h>
#include <wpi/numbers>
#include <frc/smartdashboard/Field2d.h>

#include <networktables/NetworkTableValue.h>
class Drivetrain {
    public:
        Drivetrain() {

        }
        void setupDriveMotors();
        void Drive(frc::PS4Controller* m_driverController);
        void resetEncoders();
        void updateOdometry();
    private:
        // Drive Motors
        static const int leftLeadDeviceID = 1, leftFollowDeviceID = 2, rightLeadDeviceID = 3, rightFollowDeviceID = 4;
        rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax* DriveMotors[4]= {&m_leftLeadMotor, &m_rightLeadMotor, &m_leftFollowMotor, &m_rightFollowMotor};
        rev::SparkMaxRelativeEncoder DriveEncoders[4] = {DriveMotors[0]->GetEncoder(),DriveMotors[1]->GetEncoder(),DriveMotors[2]->GetEncoder(),DriveMotors[3]->GetEncoder()};  
        frc::DifferentialDrive m_robotDrive{m_leftLeadMotor,m_rightLeadMotor};
        AHRS *ahrs;
        frc::Rotation2d rot = frc::Rotation2d{units::angle::degree_t{ahrs->GetAngle()}};
        frc::DifferentialDriveOdometry m_odometry{rot};
        frc::Field2d m_field;
};