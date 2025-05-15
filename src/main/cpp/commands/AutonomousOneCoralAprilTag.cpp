#include "commands/AutonomousOneCoralAprilTag.h"

#pragma region AutonomousOneCoralAprilTag (constructor)
/// @brief Command to place one coral in autonomous mode.
/// @param drivetrain The drivetrain subsystem.
/// @param gripper The gripper subsystem.
/// @param aprilTags The AprilTags subsystem.
AutonomousOneCoralAprilTag::AutonomousOneCoralAprilTag(GripperPoseEnum                               gripperPoseEnum,
                                                       std::function<std::string ()>                 getStartPosition,
                                                       std::function<ChassDrivePoseParameters ()>    getStartPoseParameters,
                                                       std::function<ChassDriveAprilTagParameters()> getAprilTagParameters,
                                                       Drivetrain *drivetrain, Gripper *gripper, Climb *climb)
{
    // Determine the starting position based on the selected string position ("L", "M", "R")
    if (getStartPosition().compare("M"))
    {
        AddCommands(GripperPose(GripperPoseEnum::Home, gripper),                          // Default behavior for other positions
                    ClimbSetVoltage(1.0_s, -12_V, climb),
                    //frc2::WaitCommand(0.5_s),                                             // Allow time for the gripper pose to complete
                    ChassisDrivePose(getStartPoseParameters, drivetrain),                 // Drive closedr to the reef
                    GripperPose(gripperPoseEnum, gripper),                                // Set the gripper to the appropriate level
                    frc2::WaitCommand(2.0_s),                                             // Allow time to read the AprilTag and for gripper to get to position
                    ChassisDriveToAprilTag(getAprilTagParameters, drivetrain),            // Drive to the reef
                    GripperActivate(gripper),                                             // Activate the gripper to place the coral
                    ChassisDrivePose(1_mps, -36.0_in, 0_in, 0_deg, 5_s, drivetrain));     // Move robot away from reef
    }
    else
    {
        AddCommands(GripperPose(GripperPoseEnum::Home, gripper),                          // Default behavior for other positions
                    ClimbSetVoltage(1.0_s, -12_V, climb),
                    //frc2::WaitCommand(0.5_s),                                             // Set the gripper to the appropriate level
                    ChassisDrivePose(getStartPoseParameters, drivetrain),                 // Allow time for the gripper pose to complete
                    GripperPose(gripperPoseEnum, gripper),                                // Drive closedr to the reef
                    frc2::WaitCommand(2.0_s),                                             // Allow time to read the AprilTag and for gripper to get to position
                    ChassisDriveToAprilTag(getAprilTagParameters, drivetrain),            // Drive to the reef
                    GripperActivate(gripper),                                             // Activate the gripper to place the coral
                    ChassisDrivePose(1_mps, -36.0_in, 0_in, 0_deg, 5_s, drivetrain));     // Move robot away from reef
    }
}
#pragma endregion
