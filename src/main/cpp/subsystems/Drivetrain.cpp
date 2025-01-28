#include "subsystems/DriveTrain.h"

using namespace CanConstants;

#pragma region DriveTrain (constructor)
/// @brief The Constructor for the DriveTrain class.
Drivetrain::Drivetrain()
    : m_frontLeft {SwerveFrontLeftDriveMotorCanId,  SwerveFrontLeftAngleMotorCanId,  SwerveFrontLeftAngleEncoderCanId,  ChassisConstants::kFrontLeftChassisAngularOffset},
      m_rearLeft  {SwerveRearLeftDriveMotorCanId,   SwerveRearLeftAngleMotorCanId,   SwerveRearLeftAngleEncoderCanId,   ChassisConstants::kRearLeftChassisAngularOffset},
      m_frontRight{SwerveFrontRightDriveMotorCanId, SwerveFrontRightAngleMotorCanId, SwerveFrontRightAngleEncoderCanId, ChassisConstants::kFrontRightChassisAngularOffset},
      m_rearRight {SwerveRearRightDriveMotorCanId,  SwerveRearRightAngleMotorCanId,  SwerveRearRightAngleEncoderCanId,  ChassisConstants::kRearRightChassisAngularOffset},
      m_odometry  {m_kinematics, frc::Rotation2d(units::radian_t{m_gyro.GetAngle()}),
                  {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                   m_rearLeft.GetPosition(),  m_rearRight.GetPosition()}, frc::Pose2d{}} 
{
    // Usage reporting for MAXSwerve template
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive, HALUsageReporting::kRobotDriveSwerve_MaxSwerve);
}
#pragma endregion

#pragma region Periodic
/// @brief This method will be called once periodically.
void Drivetrain::Periodic()
{
    frc::SmartDashboard::PutNumber("Gyro Angle",        GetHeading().value());
    frc::SmartDashboard::PutBoolean("Field Centricity", m_fieldCentricity);

    // Implementation of subsystem periodic method goes here.
    m_odometry.Update(frc::Rotation2d(units::radian_t{m_gyro.GetAngle()}),
                     {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
                      m_frontRight.GetPosition(), m_rearRight.GetPosition()});
}
#pragma endregion

#pragma region Drive
/// @brief Method to drive the robot chassis.
/// @param xSpeed The speed in the X dirction.
/// @param ySpeed The speed in the Y dirction.
/// @param rotation The rate of rotation.
void Drivetrain::Drive(units::meters_per_second_t  xSpeed,
                       units::meters_per_second_t  ySpeed,
                       units::radians_per_second_t rotation)
{
    // forward = frc::SmartDashboard::GetNumber("Chassis Forward", 0.0);
    // strafe  = frc::SmartDashboard::GetNumber("Chassis Strafe",  0.0);
    // angle   = frc::SmartDashboard::GetNumber("Chassis Angle",   0.0);

    frc::SmartDashboard::PutNumber("Chassis Forward", (double) xSpeed);
    frc::SmartDashboard::PutNumber("Chassis Strafe",  (double) ySpeed);
    frc::SmartDashboard::PutNumber("Chassis Angle",   (double) rotation);

    // Convert the commanded speeds into the correct units for the drivetrain
    units::meters_per_second_t  xSpeedDelivered = xSpeed.value()   * ChassisConstants::kMaxSpeed;
    units::meters_per_second_t  ySpeedDelivered = ySpeed.value()   * ChassisConstants::kMaxSpeed;
    units::radians_per_second_t rotDelivered    = rotation.value() * ChassisConstants::kMaxAngularSpeed;
  
    auto states = m_kinematics.ToSwerveModuleStates(m_fieldCentricity ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                  xSpeedDelivered, ySpeedDelivered, rotDelivered,
                  frc::Rotation2d(units::radian_t{m_gyro.GetAngle()})) : 
                  frc::ChassisSpeeds{xSpeedDelivered, ySpeedDelivered, rotDelivered});
  
    m_kinematics.DesaturateWheelSpeeds(&states, ChassisConstants::kMaxSpeed);

    auto [frontLeft, frontRight, rearLeft, rearRight] = states;

    m_frontLeft.SetState(frontLeft,   "Front Left ");  // TODO: Change to SetDesiredState
    m_frontRight.SetState(frontRight, "Front Right ");
    m_rearLeft.SetState(rearLeft,     "Rear Left ");
    m_rearRight.SetState(rearRight,   "Rear Right ");
}
#pragma endregion

#pragma region SetX
void Drivetrain::SetX() 
{
    m_frontLeft.SetDesiredState (frc::SwerveModuleState{0_mps, frc::Rotation2d{ 45_deg}});
    m_frontRight.SetDesiredState(frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
    m_rearLeft.SetDesiredState  (frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
    m_rearRight.SetDesiredState (frc::SwerveModuleState{0_mps, frc::Rotation2d{ 45_deg}});
}
#pragma endregion

#pragma region ResetEncoders
void Drivetrain::ResetEncoders() 
{
    m_frontLeft.ResetEncoders();
    m_rearLeft.ResetEncoders();
    m_frontRight.ResetEncoders();
    m_rearRight.ResetEncoders();
}

#pragma region SetModuleStates
void Drivetrain::SetModuleStates(wpi::array<frc::SwerveModuleState, ChassisConstants::NumberOfSwerveModules> desiredStates) 
{
    m_kinematics.DesaturateWheelSpeeds(&desiredStates, ChassisConstants::kMaxSpeed);
    m_frontLeft.SetDesiredState (desiredStates[0]);
    m_frontRight.SetDesiredState(desiredStates[1]);
    m_rearLeft.SetDesiredState  (desiredStates[2]);
    m_rearRight.SetDesiredState (desiredStates[3]);
}
#pragma endregion

#pragma region GetHeading
/// @brief Method to get the robot heading.
/// @return The robot heading.
units::degree_t Drivetrain::GetHeading()
{
    // Return the robot heading
        return (units::degree_t) m_gyro.GetAngle();
}
#pragma endregion

#pragma region ZeroHeading
/// @brief Method to zero the robot heading.
void Drivetrain::ZeroHeading() 
{ 
    m_gyro.Reset(); 
}
#pragma endregion

#pragma region GetTurnRate
/// @brief Method to determine the turn rate of the chassis.
/// @return The chassis turn rate.
double Drivetrain::GetTurnRate() 
{
    // TODO: Ensure the return value should be negative
    return -m_gyro.GetRate();;
}
#pragma endregion

#pragma region GetPose
/// @brief Method to get the pose of the chassis.
/// @return The chassis pose.
frc::Pose2d Drivetrain::GetPose() 
{ 
    return m_odometry.GetPose(); 
}
#pragma endregion

#pragma region ResetOdometry
/// @brief Method to reset the chassis odometry.
/// @param pose TODO:: ???
void Drivetrain::ResetOdometry(frc::Pose2d pose) 
{
    m_odometry.ResetPosition(GetHeading(), {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                                            m_rearLeft.GetPosition(),  m_rearRight.GetPosition()}, pose);
}
#pragma endregion

#pragma region SetFieldCentricity
/// @brief Method to set the robot control field centricity.
/// @param fieldCentric Boolean to indicate if the robor control should be field centric.
void Drivetrain::SetFieldCentricity(bool fieldCentric)
{
    // Set the field centric member variable
    m_fieldCentricity = fieldCentric;
}
#pragma endregion

#pragma region GetFieldCentricity
/// @brief Method to set the field centricity.
/// @return The field centricity setting.
bool Drivetrain::GetFieldCentricity()
{
    // Return the field centricity setting
    return m_fieldCentricity;
}
#pragma endregion

#pragma region SetWheelAnglesToZero
/// @brief Method to set the swerve wheel to the absoulute encoder angle then zero the PID controller angle.
void Drivetrain::SetWheelAnglesToZero()
{
    // Set the swerve wheel angles to zero
    // m_swerveModule[SwerveConstants::FrontRightIndex]->SetWheelAngleToForward(SwerveConstants::FrontRightForwardAngle);
    // m_swerveModule[SwerveConstants::FrontLeftIndex]->SetWheelAngleToForward(SwerveConstants::FrontLeftForwardAngle);
    // m_swerveModule[SwerveConstants::RearRightIndex]->SetWheelAngleToForward(SwerveConstants::RearRightForwardAngle);
    // m_swerveModule[SwerveConstants::RearLeftIndex]->SetWheelAngleToForward(SwerveConstants::RearLeftForwardAngle);
}
#pragma endregion
