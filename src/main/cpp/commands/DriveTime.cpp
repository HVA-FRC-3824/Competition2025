#include "commands/DriveTime.h"

DriveTime::DriveTime(units::second_t time, double speed, Drivetrain *drivetrain) : m_time(time), m_speed(speed), m_drivetrain(drivetrain)
{
    // Set the command name
    SetName("DriveTime");

    // Declare subsystem dependencies
    AddRequirements(drivetrain);
}

/// @brief Called just before this Command runs the first time.
void DriveTime::Initialize()
{
    // Remember the field centric setting
    m_fieldCentricity = m_drivetrain->GetFieldCentricity();
    
    // Do not use field coordinates
    m_drivetrain->SetFieldCentricity(false);

    // Get the start time
    m_startTime = frc::GetTime();
}

/// @brief Called repeatedly when this Command is scheduled to run.
void DriveTime::Execute()
{
    // Start driving
    m_drivetrain->Drive(m_speed, 0.0, 0.0, 0.0);
}

/// @brief Indicates if the command has completed. Make this return true when this Command no longer needs to run execute().
/// @return True is the command has completed.
bool DriveTime::IsFinished()
{
    // Determine if the sequence is complete
    if (frc::GetTime() - m_startTime > m_time)
        return true;

    // Still driving
    return false;    
}

/// @brief Called once after isFinished returns true.
/// @param interrupted Indicated that the command was interrupted.
void DriveTime::End(bool interrupted)
{
    // Stop the move
    m_drivetrain->Drive(0.0, 0.0, 0.0, 0.0);

    // Restore the field centricity
    m_drivetrain->SetFieldCentricity(m_fieldCentricity);
}