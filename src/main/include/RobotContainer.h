#pragma once

#pragma region Includes
#include <utility>

#include <frc/filter/SlewRateLimiter.h>
#include <frc/Joystick.h>
#include <frc/MathUtil.h>
#include <frc/PowerDistribution.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/XboxController.h>

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SwerveControllerCommand.h>

#include <cameraserver/CameraServer.h>

// Subsystems
#include "subsystems/Climb.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/Gripper.h"
#include "subsystems/Leds.h"

// Commands
#include "commands/AutonomousComplex.h"
#include "commands/AutonomousCoralAndAlgae.h"
#include "commands/AutonomousDoNothing.h"
#include "commands/AutonomousLed.h"
#include "commands/AutonomousOneCoral.h"
#include "commands/AutonomousOneCoralAprilTag.h"
#include "commands/AutonomousParallel.h"
#include "commands/AutonomousRaceGroup.h"
#include "commands/ChassisDrive.h"
#include "commands/ChassisDrivePose.h"
#include "commands/ChassisDriveSerpentine.h"
#include "commands/ChassisDriveTime.h"
#include "commands/ChassisDriveToAprilTag.h"
#include "commands/ChassisSetSwerveWheelAnglesToZero.h"
#include "commands/GripperActivate.h"
#include "commands/GripperPose.h"
#include "commands/SetLeds.h"

#include "Constants.h"
#include "ConstantsChassisPoseAprilTag.h"
#include "ConstantsChassisPoseAutonomous.h"
#include "ConstantsGripperPoseCoral.h"
#pragma endregion

/// @brief Class to instantiate the robot subsystems and commands along with the operator controls
class RobotContainer
{
    public:

        // Method that returns a pointer to the singleton instance of the RobotContainer class
        static RobotContainer       *GetInstance();

        // Method to get a pointer to the selected autonomous command
        frc2::Command               *GetAutonomousCommand();

        // Method to set the swerve wheels to starting position based on the absolute encoder
        void                         SetSwerveWheelAnglesToZero();

        // Methods to get a reference to the robot joysticks
        frc::Joystick               *GetDriverController();
        frc::XboxController         *GetOperatorController();

        units::meters_per_second_t   Forward();
        units::meters_per_second_t   Strafe();
        units::radians_per_second_t  Angle();

        GripperWheelState            PotentiometerWheelVoltage();

        frc::Pose2d                  GetChassisPose();
        void                         ReverseChassisGryo();

        ChassDriveAprilTagParameters GetChassisDriveToAprilTagParameters();

        Gripper                     *GetGripper();

        frc::PowerDistribution      *GetPowerDistribution();

    private:

        // Private class constructor to configure the robot and SmartDashboard configuration
        RobotContainer();

        // Method to bind the joystick controls to the robot commands
        void                     ConfigureButtonBindings();

        void                     ConfigureDriverControls();
        void                     ConfigureCoralPoseControls();
        void                     ConfigureAlgaePoseControls();
        void                     ConfigureGripperControls();
        void                     ConfigureClimberControls();

        double                   GetThrottleRange();
        double                   GetExponentialValue(double joystickValue, double exponent);

        std::string              GetStartPosition();

        ChassDrivePoseParameters GetAutonomousOneCoralParameters(units::length::inch_t distanceXOffset, units::length::inch_t distanceYOffset);
        ChassDrivePoseParameters GetAutonomousOneCoralAprilTagParameters();
        ChassDrivePoseParameters GetAutonomousTwoCoralAprilTagParameters();

        // Singleton reference to the class (returned by the GetInstance Method)
        static RobotContainer                *m_robotContainer;

        // Instantiate the robot subsystems
        Climb                                 m_climb;
        Drivetrain                            m_drivetrain;
        Gripper                               m_gripper;
        Leds                                  m_leds;

        // Joysticks
        frc::Joystick                         m_driverController{ConstantsController::DriverControllerUsbPort};
        frc::XboxController                   m_operatorController{ConstantsController::JoystickOperatorUsbPort};

        // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        frc::SlewRateLimiter<units::scalar>   m_xspeedLimiter{3 / 1_s};
        frc::SlewRateLimiter<units::scalar>   m_yspeedLimiter{3 / 1_s};
        frc::SlewRateLimiter<units::scalar>   m_rotLimiter   {3 / 1_s};

        // Create the command to set the swerve wheel angles to zero based on the absolute encoder
        ChassisSetSwerveWheelAnglesToZero    *m_swerveWheelAnglesToZero = new ChassisSetSwerveWheelAnglesToZero(&m_drivetrain);

        // Autonomous command chooser
        frc::SendableChooser<frc2::Command*>  m_autonomousChooser;

        // Autonomous starting position command chooser
        frc::SendableChooser<std::string>     m_startingPositionChooser;

        frc::PowerDistribution                m_powerDistribution;

        // cs::VideoSink                         m_server;
        // cs::UsbCamera                         m_usbCamera;
        // cs::VideoSink                         m_limelightFeed;

        // bool                                  m_usbCameraActive = false;
};
