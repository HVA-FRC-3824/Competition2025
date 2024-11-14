
#include "RobotContainer.h"

#include <frc2/command/ParallelRaceGroup.h>
#include <frc/smartdashboard/SmartDashboard.h>

RobotContainer *RobotContainer::m_robotContainer = NULL;

RobotContainer::RobotContainer()
{
    // Smartdashboard Subsystems
    frc::SmartDashboard::PutData(&m_leds);
    frc::SmartDashboard::PutData(&m_drivetrain);

    // SmartDashboard Buttons
    frc::SmartDashboard::PutData("Autonomous Command", new AutonomousCommand());

    frc::SmartDashboard::PutData("SetLeds: Off", new SetLeds(LedMode::LedOff, &m_leds));
    frc::SmartDashboard::PutData("SetLeds: Rainbow", new SetLeds(LedMode::Rainbow, &m_leds));



    frc::SmartDashboard::PutData("ChassisDrive: Stop", new ChassisDrive(0, 0, &m_drivetrain));

    frc::SmartDashboard::PutData("DriveDistance: OneFoot", new DriveDistance(1, &m_drivetrain));
    frc::SmartDashboard::PutData("DriveDistance: TwoFeet", new DriveDistance(2, &m_drivetrain));

    ConfigureButtonBindings();

    m_drivetrain.SetDefaultCommand(ChassisDrive(0, 0, &m_drivetrain));
    m_leds.SetDefaultCommand(SetLeds(LedMode::Rainbow, &m_leds));

    m_chooser.AddOption("DriveDistance: OneFoot", new DriveDistance(1, &m_drivetrain));
    m_chooser.AddOption("DriveDistance: TwoFeet", new DriveDistance(2, &m_drivetrain));
    m_chooser.SetDefaultOption("Autonomous Command", new AutonomousCommand());

    frc::SmartDashboard::PutData("Auto Mode", &m_chooser);
}

RobotContainer *RobotContainer::GetInstance()
{
    if (m_robotContainer == NULL)
    {
        m_robotContainer = new RobotContainer();
    }
    return (m_robotContainer);
}

void RobotContainer::ConfigureButtonBindings()
{
    frc2::JoystickButton m_setLEDsOff{&m_joystickController, 1};
    frc2::JoystickButton m_setLEDsRainbow{&m_joystickController, 2};

    m_setLEDsOff.OnTrue(SetLeds(0, &m_leds).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    m_setLEDsRainbow.OnTrue(SetLeds(1, &m_leds).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));


}

frc::Joystick *RobotContainer::getJoystickOperator()
{
    return &m_joystickOperator;
}

frc::Joystick *RobotContainer::getJoystickController()
{
    return &m_joystickController;
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
    // The selected command will be run in autonomous
    return m_chooser.GetSelected();
}
