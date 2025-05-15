#include "commands/AutonomousCoralAndAlgae.h"

using namespace ConstantsChassisPoseAutonomous;

#pragma region AutonomousCoralAndAlgae (constructor)
/// @brief Command to place one coral in autonomous mode.
AutonomousCoralAndAlgae::AutonomousCoralAndAlgae(GripperPoseEnum                               gripperPoseEnum,
                                                 std::function<std::string()>                  getStartingPosition,
                                                 std::function<ChassDrivePoseParameters ()>    getStartPoseParameters,
                                                 std::function<ChassDriveAprilTagParameters()> getAprilTagParameters,
                                                 Drivetrain *drivetrain, Gripper *gripper)
{
    // AddCommands(AutonomousOneCoralAprilTag(gripperPoseEnum, getStartingPosition, getStartPoseParameters, getAprilTagParameters, drivetrain, gripper),  // Do all the coral stuff
    //             ChassisDrivePose(2_mps, -24.0_in, 0.0_m, 0.0_deg, 15_s, drivetrain),                                   // Move robot away from reef
    //             GripperPose(GripperPoseEnum::AlgaeHigh, gripper),                                                      // Set the gripper pose for the algae
    //             ChassisDriveToAprilTag(getAprilTagParameters, drivetrain),                                             // Drive to the reef for algae
    //             GripperActivate(gripper),                                                                              // Get the algae
    //             ChassisDrivePose(2_mps, -24.0_in, 0.0_m, 0.0_deg, 15_s, drivetrain),                                   // Move robot away from reef
    //             ChassisDrivePose(AlgaeAndCoralSpeed, AlgaeAndCoralXDistance, AlgaeAndCoralYDistance,                   // Drive to barge
    //                              AlgaeAndCoralAngleChange, AutonomousTimeOut, drivetrain),
    //             GripperPose(GripperPoseEnum::AlgaeBarge, gripper),                                                     // Set the gripper pose for the barge
    //             GripperActivate(gripper)                                                                               // Place the algae into the barge
    //             );
}
#pragma endregion
