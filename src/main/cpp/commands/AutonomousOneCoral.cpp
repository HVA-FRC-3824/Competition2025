#include "commands/AutonomousOneCoral.h"

#pragma region AutonomousOneCoral (constructor)
/// @brief Command to place one coral in autonomous mode.
/// @param drivetrain The drivetrain subsystem.
/// @param gripper The gripper subsystem.
AutonomousOneCoral::AutonomousOneCoral(GripperPoseEnum gripperPoseEnum, std::function<ChassDrivePoseParameters ()> getParameters,
                                       Drivetrain *drivetrain, Gripper *gripper)
{
    // Run the command sequence
    AddCommands(GripperPose(GripperPoseEnum::Home, gripper),  // Set the gripper pose to Home
                frc2::WaitCommand(1.0_s),                     // Let Activate get the coral on the reef a little
                GripperPose(gripperPoseEnum, gripper),        // Set the gripper to the appropiate level
                frc2::WaitCommand(2.0_s),                     // Allow time for the gripper pose to complete
                ChassisDrivePose(getParameters, drivetrain),  // Drive to the reef
                frc2::WaitCommand(1.0_s),                     // Allow time for the gripper pose to complete
                GripperActivate(gripper),                     // Place the coral
                frc2::WaitCommand(0.15_s),                    // Let Activate get the coral on the reef a little
                ChassisDrivePose(getParameters().Speed,       // Move robot away from reef
                                 -12.0_in, 0.0_m, 0.0_deg,
                                 15_s, drivetrain)
    );
}
#pragma endregion
