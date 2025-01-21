#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivetrain.h"

class ChassisSetSwerveWheelAnglesToZero : public frc2::CommandHelper<frc2::Command, ChassisSetSwerveWheelAnglesToZero>
{
    public:

        ChassisSetSwerveWheelAnglesToZero(Drivetrain *drivetrain);

        void Initialize() override;
        bool IsFinished() override;

    private:

        Drivetrain *m_drivetrain;  // Pointer to the chassis set the swerve wheel angles to zero
};