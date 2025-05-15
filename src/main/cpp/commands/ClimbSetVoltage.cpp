// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimbSetVoltage.h"

ClimbSetVoltage::ClimbSetVoltage(units::time::second_t timeToRun,
                                 units::voltage::volt_t voltage,
                                 Climb* climb) :
                                 m_timeToRun(timeToRun),
                                 m_voltage(voltage),
                                 m_climb(climb)

{
  
}

// Called when the command is initially scheduled.
void ClimbSetVoltage::Initialize() 
{
    // Get the start time
    m_startTime = frc::GetTime();

    m_climb->SetVoltage(m_voltage);
}

// Called repeatedly when this Command is scheduled to run
void ClimbSetVoltage::Execute() {}

// Called once the command ends or is interrupted.
void ClimbSetVoltage::End(bool interrupted) 
{
  m_climb->SetVoltage(0_V);
}

// Returns true when the command should end.
bool ClimbSetVoltage::IsFinished() 
{
    // Determine if the sequence is complete
    if (frc::GetTime() - m_startTime > m_timeToRun)
        return true;

    // Still driving
    return false;
}
