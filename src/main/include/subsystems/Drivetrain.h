#pragma once

#include <numbers>

#include "studica/AHRS.h"

#include <hal/FRCUsageReporting.h>

#include <frc/ADIS16470_IMU.h>
#include <frc/AnalogGyro.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>

#include "SwerveModule.h"

#include "Constants.h"
#include "ConstantsCanIds.h"

#pragma region DrivetrainConstants
namespace DrivetrainConstants
{
    // Chassis configuration
    constexpr auto TrackWidth               = 25_in;  // Distance between centers of right and left wheels on robot
    constexpr auto WheelBase                = 25_in;  // Distance between centers of front and back wheels on robot

    constexpr auto MaxSpeed                 = 4.8_mps;
    constexpr auto MaxAngularSpeed          = std::numbers::pi * 2_rad_per_s;
}
#pragma endregion

#pragma region ChassisPoseConstants
namespace ChassisPoseConstants
{
    constexpr auto   MaxSpeed               = 2_mps;
    constexpr auto   MaxAcceleration        = 2_mps_sq;
    constexpr auto   MaxAngularSpeed        = 3.142_rad_per_s;
    constexpr auto   MaxAngularAcceleration = 3.142_rad_per_s_sq;

    constexpr double PXController           = 4.0;
    constexpr double PYController           = 4.0;
    constexpr double PProfileController     = 5.0;

    extern const frc::TrapezoidProfile<units::radians>::Constraints ThetaControllerConstraints;
}
#pragma endregion

class Drivetrain : public frc2::SubsystemBase
{
    public:

        explicit        Drivetrain();

        void            Periodic() override;

        void            Drive(units::meters_per_second_t  xSpeed,
                              units::meters_per_second_t  ySpeed,
                              units::radians_per_second_t rotation);

        void            SetX();                                 // Sets the wheels into an X formation to prevent movement

        void            ResetDriveEncoders();                   // Resets the drive encoders to currently read a position of 0

        void            SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

        frc::Rotation2d GetRotation2d();                        // Returns the rotation of the robot
        units::degree_t GetHeading();                           // Returns the heading of the robot

        void            ZeroHeading();                          // Zeroes the heading of the robot
        void            ZeroHeadingReverse();                   // Zeroes the heading and reverses the gyro (180 degrees)
        void            ReverseHeading();                       // Reverses the gyro (180 degress)

        frc::Pose2d     GetPose();                              // Returns the currently-estimated pose of the robot

        void            ResetPositionToOrgin();                 // Resets the odometry to the orgin

        void            SetFieldCentricity(bool fieldCentric);  // Sets the field centricity
        bool            GetFieldCentricity();                   // Reads the field centricity

        void            SetWheelAnglesToZero();                 // Sets the wheels to forward based on the absolute encoder

        units::inch_t   GetDistance();

        // Swerve module order for kinematics calculations
        //
        //         Front          Translation2d Coordinates
        //   FL +----------+ FR              ^ X
        //      | 0      1 |                 |
        //      |          |            Y    |
        //      |          |          <------+-------
        //      | 2      3 |                 |
        //   RL +----------+ RR              |

        frc::SwerveDriveKinematics<4> m_kinematics{
            frc::Translation2d{ DrivetrainConstants::WheelBase / 2,  DrivetrainConstants::TrackWidth / 2},   // Front Left
            frc::Translation2d{ DrivetrainConstants::WheelBase / 2, -DrivetrainConstants::TrackWidth / 2},   // Front Right
            frc::Translation2d{-DrivetrainConstants::WheelBase / 2,  DrivetrainConstants::TrackWidth / 2},   // Rear Left
            frc::Translation2d{-DrivetrainConstants::WheelBase / 2, -DrivetrainConstants::TrackWidth / 2}};  // Rear Right

    private:

        studica::AHRS               m_gyro{studica::AHRS::NavXComType::kMXP_SPI};  // The gyro sensor

        bool                        m_fieldCentricity = true;                      // Field centricity flag

        SwerveModule                m_frontLeft;
        SwerveModule                m_frontRight;
        SwerveModule                m_rearLeft;
        SwerveModule                m_rearRight;

        // Odometry class for tracking robot pose for the swerve modules modules
        frc::SwerveDriveOdometry<4> m_odometry;
};
