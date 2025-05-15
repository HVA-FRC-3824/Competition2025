// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Climb.h"

class ClimbSetVoltage : public frc2::CommandHelper<frc2::Command, ClimbSetVoltage> 
{
    public:
  
        ClimbSetVoltage(units::time::second_t timeToRun, 
                        units::voltage::volt_t voltage, 
                        Climb* climb);

        void Initialize() override;

        void Execute() override;

        void End(bool interrupted) override;

        bool IsFinished() override;
    
    private:
        Climb* m_climb;

        units::voltage::volt_t m_voltage;

        units::time::second_t m_timeToRun;
        units::second_t m_startTime;
};
