#include "commands/ChassisSetSwerveWheelAnglesToZero.h"

#pragma region ChassisSetSwerveWheelAnglesToZero (constructor)
/// @brief Command to set the swerve wheels to the starting position based on the absolute encode.
ChassisSetSwerveWheelAnglesToZero::ChassisSetSwerveWheelAnglesToZero(Drivetrain *drivetrain) : m_drivetrain(drivetrain)
{
    // Set the command name
    SetName("ChassisSetSwerveWheelAnglesToZero");

    // Declare subsystem dependencies
    AddRequirements({m_drivetrain});
}
#pragma endregion

#pragma region Execute
/// @brief Called repeatedly when this Command is scheduled to run.
void ChassisSetSwerveWheelAnglesToZero::Execute()
{
    // Set the swerve wheel angles to zero
    m_drivetrain->SetWheelAnglesToZero();
}
#pragma endregion

#pragma region IsFinished
/// @brief Returns true when the command should end.
/// @return True when the command should end.
bool ChassisSetSwerveWheelAnglesToZero::IsFinished()
{
    // Execute only runs once
    return true;
}
#pragma endregion
