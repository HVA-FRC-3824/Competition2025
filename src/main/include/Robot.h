#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>

#include "Constants.h"
#include "RobotContainer.h"

class Robot : public frc::TimedRobot
{
public:

  void RobotInit()          override;
  void RobotPeriodic()      override;
  void AutonomousInit()     override;
  void AutonomousPeriodic() override;
  void TeleopInit()         override;
  void TeleopPeriodic()     override;
  void DisabledInit()       override;
  void DisabledPeriodic()   override;
  void TestInit()           override;
  void TestPeriodic()       override;
  void SimulationInit()     override;
  void SimulationPeriodic() override;

  frc::Joystick* operatorJoystick;

  Leds* LedSubSystem;

private:

  frc2::Command* m_autonomousCommand = nullptr;

  RobotContainer* m_container = RobotContainer::GetInstance();
};
