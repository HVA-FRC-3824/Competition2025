#pragma once

#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SwerveControllerCommand.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "LimelightHelpers.h"

#include "subsystems/Drivetrain.h"

#include "commands/ChassisDrivePose.h"

struct ChassDriveAprilTagParameters
{
    bool                      ValidPose;
    bool                      ReefRightSide;
    bool                      FieldCentricity;
    ChassDrivePoseParameters  PoseParameters;
};

class ChassisDriveToAprilTag : public frc2::CommandHelper<frc2::Command, ChassisDriveToAprilTag>
{
    public:

        explicit ChassisDriveToAprilTag(units::meters_per_second_t speed,
                                        units::meter_t             distanceOffsetX,
                                        units::meter_t             distanceOffsetY,
                                        units::degree_t            angleOffset,
                                        units::time::second_t      timeoutTime,
                                        Drivetrain                *drivetrain);

        explicit ChassisDriveToAprilTag(std::function<ChassDriveAprilTagParameters()> getParameters, Drivetrain *drivetrain);

        void     Initialize()          override;
        void     Execute()             override;
        bool     IsFinished()          override;
        void     End(bool interrupted) override;

    private:

        bool                              m_readParameters;             // Indicates the parameters should be read from the lambda function

        units::meters_per_second_t        m_speed;                      // The speed of the chassis
        units::time::second_t             m_timeoutTime;                // The command time-out time
        units::meter_t                    m_distanceOffsetX;            // The distance offset in the X direction
        units::meter_t                    m_distanceOffsetY;            // The distance offset in the Y direction
        units::degree_t                   m_angleOffset;                // The angle offset
        Drivetrain                       *m_drivetrain;                 // The drivetrain subsystem

        units::second_t                   m_startTime;                  // The start of the drive time
        bool                              m_finished;                   // Indicates the command is finished
        frc2::SwerveControllerCommand<4> *m_swerveControllerCommand;    // The swerve controller command

        std::function<ChassDriveAprilTagParameters()> m_getParameters;  // The lambda function to get the parameters
};
