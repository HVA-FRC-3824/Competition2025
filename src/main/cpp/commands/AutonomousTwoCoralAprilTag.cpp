// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutonomousTwoCoralAprilTag.h"

#include "subsystems/Drivetrain.h"
#include "subsystems/Gripper.h"

#include "commands/ChassisDrivePose.h"
#include "commands/ChassisDriveTime.h"
#include "commands/ChassisDriveToAprilTag.h"
#include "commands/GripperActivate.h"
#include "commands/GripperPose.h"
#include "commands/AutonomousOneCoralAprilTag.h"

AutonomousTwoCoralAprilTag::AutonomousTwoCoralAprilTag(GripperPoseEnum                               gripperPoseEnum,
                                                       std::function<std::string ()>                 getStartingPosition,
                                                       std::function<ChassDrivePoseParameters ()>    getOneCoralParemters,
                                                       std::function<ChassDriveAprilTagParameters()> getSecondCoralParemters,
                                                       std::function<ChassDriveAprilTagParameters()> getAprilTagParameters,
                                                       std::function<ChassDriveAprilTagParameters()> getStationParemters,
                                                       Drivetrain *drivetrain, Gripper *gripper) 
{
    // AddCommands(AutonomousOneCoralAprilTag(gripperPoseEnum, getStartingPosition, getOneCoralParemters, getAprilTagParameters, drivetrain, gripper),
    //             GripperPose(GripperPoseEnum::CoralStation, gripper),
    //             //ChassisDrivePose() // move to point to station AprilTag
    //             frc2::WaitCommand(0.3_s) // Pause to see AprilTag
    //             // Move to station AprilTag
    //             // Wait to get Coral
    //             // Move to point to reefApriltag
    //             // Move to reef
    //             // GripperActivate
    //             // Back up 36 inches

    //             //ChassisDrivePose(2_mps, -12.0_in, 0_in, 0_deg, 5_s, drivetrain));     // Move robot away from reef
    // );
}

