#pragma once

#pragma region Includes
#include <iostream>
#include <string>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Translation2d.h>

#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_jerk.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#pragma endregion

#pragma region CanConstants
namespace CanConstants
{
    const     auto CanBus                            = "rio";

    constexpr auto SwerveFrontRightDriveMotorCanId   =  3;
    constexpr auto SwerveFrontRightAngleMotorCanId   =  2;
    constexpr auto SwerveFrontRightAngleEncoderCanId = 12;

    constexpr auto SwerveFrontLeftDriveMotorCanId    =  6;
    constexpr auto SwerveFrontLeftAngleMotorCanId    =  5;
    constexpr auto SwerveFrontLeftAngleEncoderCanId  =  4;

    constexpr auto SwerveRearLeftDriveMotorCanId     =  9;
    constexpr auto SwerveRearLeftAngleMotorCanId     =  8;
    constexpr auto SwerveRearLeftAngleEncoderCanId   =  7;

    constexpr auto SwerveRearRightDriveMotorCanId    = 11;
    constexpr auto SwerveRearRightAngleMotorCanId    = 10;
    constexpr auto SwerveRearRightAngleEncoderCanId  = 13;

    constexpr int ElevatorMotorCanId                 = 20;
}
#pragma endregion

#pragma region ChassisConstants
namespace ChassisConstants
{
    constexpr auto NumberOfSwerveModules           =     4;

    constexpr auto ChassisLength                   =   100;  // TODO: Set properly
    constexpr auto ChassisWidth                    =   100;

    constexpr auto MotorConfigurationAttempts      =     5;

    constexpr auto SwerveDriveMaxAmperage          =  60_A;

    constexpr auto SwerveAngleMaxAmperage          =    30;

    constexpr auto SwerveMotorRevolutions          =    21.5;                                 // The number of motor revolutions per wheel revolutions
    constexpr auto SwerveDegreesToMotorRevolutions = 180.0 / (SwerveMotorRevolutions / 2.0);  // Degrees to motor revolutions
}
#pragma endregion

#pragma region SwerveConstants
namespace SwerveConstants
{
    constexpr auto FrontRightIndex        = 0;
    constexpr auto FrontLeftIndex         = 1;
    constexpr auto RearRightIndex         = 3;
    constexpr auto RearLeftIndex          = 2;

    constexpr auto FrontRightForwardAngle =  0.301 * 360_deg;
    constexpr auto FrontLeftForwardAngle  = -0.464 * 360_deg;
    constexpr auto RearRightForwardAngle  = -0.064 * 360_deg;
    constexpr auto RearLeftForwardAngle   = -0.022 * 360_deg;

    constexpr auto P                      = 0.025;
    constexpr auto I                      = 0.000;
    constexpr auto D                      = 0.010;    
}
#pragma endregion

#pragma region ElevatorContants
namespace ElevatorContants
{
    constexpr auto S                               = 0.25;             // Static Friction: Add [voltage] output to overcome static friction
    constexpr auto V                               = 0.12;             // Velocity:        A velocity target of 1 rps results in [voltage] output
    constexpr auto A                               = 0.01;             // Acceleration:    An acceleration of 1 rps/s requires 0.01 V output
    constexpr auto P                               = 60.0;             // Proportional:    A position error of 0.2 rotations results in 12 V output
    constexpr auto I                               =  0.0;             // Integral:        No output for integrated error
    constexpr auto D                               =  0.5;             // Differential     A velocity error of 1 rps results in 0.5 V output

    constexpr auto SensorToMechanismRatio          = 12.8;             // 12.8 rotor rotations per mechanism rotation

    constexpr auto MotionMagicCruiseVelocity       = 5_tps;            // 5 (mechanism) rotations per second cruise
    constexpr auto MotionMagicAcceleration         = 10_tr_per_s_sq;   // Take approximately 0.5 seconds to reach max vel
    constexpr auto MotionMagicJerk                 = 100_tr_per_s_cu;  // Take approximately 0.1 seconds to reach max accel   

    constexpr auto PositionToTurnsConversionFactor = 1.0;
}
#pragma endregion

#pragma region ControllerConstants
namespace ControllerConstants
{
    constexpr auto DriverControllerUsbPort =   0;
    constexpr auto JoystickOperatorUsbPort =   1;

    constexpr auto JoystickForwardIndex    =   1;
    constexpr auto JoystickStrafeIndex     =   0;
    constexpr auto JoystickAngleIndex      =   2;  // 4 for xbox controller, 2 for extreme 3d controller(stick controller)

    constexpr auto JoystickAngleDeadZone   = 0.1;
    constexpr auto JoystickDeadZone        = 0.1;

    constexpr auto ExponentForward         = 2.0;
    constexpr auto ExponentStrafe          = 2.0;
    constexpr auto ExponentAngle           = 2.0;
}
#pragma endregion

#pragma region Extreme3DContants
namespace Extreme3DContants
{
    constexpr auto HandleLowerLeft  = 3;
    constexpr auto HandleLowerRight = 4;
}
#pragma endregion

#pragma region xBoxConstants
namespace XBoxConstants
{
    constexpr auto A                 =   1;
    constexpr auto B                 =   2;
    constexpr auto X                 =   3;
    constexpr auto Y                 =   4;
    constexpr auto LeftBumper        =   5;
    constexpr auto RightBumper       =   6;
    constexpr auto Back              =   7;
    constexpr auto Start             =   8;
    constexpr auto LeftStickButton   =   9;
    constexpr auto RightStickButton  =  10;

    constexpr auto Pov_0             =   0;
    constexpr auto Pov_45            =  45;
    constexpr auto Pov_90            =  90;
    constexpr auto Pov_135           = 135;
    constexpr auto Pov_180           = 180;
    constexpr auto Pov_225           = 225;
    constexpr auto Pov_270           = 270;
    constexpr auto Pov_315           = 315;
}
#pragma endregion

#pragma region ApriltagConstants
namespace ApriltagConstants
{
    // Magic camera values:
    // Source: https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21
    constexpr auto CameraWidthInPixels     = 699.3778103158814;
    constexpr auto CameraHeightInPixels    = 677.7161226393544;
    constexpr auto CameraCenterXInPixels   = 345.6059345433618;
    constexpr auto CameraCenterYInPixels   = 207.12741326228522;

    constexpr auto CameraResolutionWidth   = 640;
    constexpr auto CameraResolutionHeight  = 480;

    constexpr auto AprilTagLineWitdh       =   2;
    constexpr auto NumberOfAprilTagCorners =   4;
    constexpr auto NumberOfBitsCorrected   =   1;

    constexpr auto LengthOfTagsInches      = 6.5;
}
#pragma endregion

#pragma region LedConstants
namespace LedConstants
{
    constexpr auto Length      =  40;  // The length of the LED string
    constexpr auto PwmPort     =   5;
    constexpr auto Brightness  = 0.5;

    constexpr auto Red         = 255;
    constexpr auto Green       = 255;
    constexpr auto Blue        = 255;

    constexpr auto StrobeDelay =  20;  // The delay between strobe flashes
    constexpr auto HvaDelay    =  20;  // The delay between HVA color changes
}
#pragma endregion
