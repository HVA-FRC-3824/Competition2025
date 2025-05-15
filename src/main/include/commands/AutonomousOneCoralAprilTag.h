#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/RunCommand.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/Gripper.h"
#include "subsystems/Climb.h"

#include "commands/ChassisDrivePose.h"
#include "commands/ChassisDriveTime.h"
#include "commands/ChassisDriveToAprilTag.h"
#include "commands/GripperActivate.h"
#include "commands/GripperPose.h"
#include "commands/ClimbSetVoltage.h"

#include "Constants.h"

class AutonomousOneCoralAprilTag : public frc2::CommandHelper<frc2::SequentialCommandGroup, AutonomousOneCoralAprilTag>
{
    public:

        explicit AutonomousOneCoralAprilTag(GripperPoseEnum                               gripperPoseEnum,
                                            std::function<std::string()>                  getStartingPosition,
                                            std::function<ChassDrivePoseParameters ()>    getStartPoseParameters,
                                            std::function<ChassDriveAprilTagParameters()> getAprilTagParameters,
                                            Drivetrain *drivetrain, Gripper *gripper, Climb *climb);
};
