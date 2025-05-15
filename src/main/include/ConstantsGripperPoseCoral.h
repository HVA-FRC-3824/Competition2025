#pragma once

#pragma region Includes
#include <iostream>
#include <numbers>
#include <string>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Translation2d.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "subsystems/Gripper.h"
#pragma endregion

#pragma region ConstantsGripperPoseCoral
namespace ConstantsGripperPoseCoral
{
    constexpr auto CoralGroundPickupVoltage      = 4.0_V;
    constexpr auto CoralGripVoltage              = 0.5_V;

    constexpr auto HomeElevator                  = 0.05_m;
    constexpr auto HomeArmAngle                  = 25_deg;
    constexpr auto HomeWristAngle                = 0.0_deg;
    constexpr auto HomeGripperBothWheels         = true;
    constexpr auto HomeGripperVoltage            = CoralGripVoltage;

    constexpr auto GroundElevator                = 0.114432_m;
    constexpr auto GroundArmAngle                = 138.765_deg;
    constexpr auto GroundWristAngle              = 180.0_deg;
    constexpr auto GroundGripperBothWheels       = false;
    constexpr auto GroundGripperVoltage          = CoralGroundPickupVoltage;

    constexpr auto StationElevator               = 0.053983_m;    //0.055263_m;//0.095_m;
    constexpr auto StationArmAngle               = -5.0_deg;  //6.435547_deg;//-19.75_deg;
    constexpr auto StationWristAngle             = 180.0_deg;
    constexpr auto StationGripperBothWheels      = true;
    constexpr auto StationGripperVoltage         = 5.0_V;  // Verify the voltage

    constexpr auto L1Elevator                    = 0.450036_m;
    constexpr auto L1ArmAngle                    = 97.778320_deg;
    constexpr auto L1WristAngle                  = 180.0_deg;
    constexpr auto L1GripperBothWheels           = true;
    constexpr auto L1GripperVoltage              = CoralGripVoltage;

    constexpr auto AutonomousL1Elevator          = L1Elevator;
    constexpr auto AutonomousL1ArmAngle          = L1ArmAngle;
    constexpr auto AutonomousL1WristAngle        = 0.0_deg;
    constexpr auto AutonomousL1GripperBothWheels = true;
    constexpr auto AutonomousL1GripperVoltage    = CoralGripVoltage;

    constexpr auto L2Elevator                    = 0.369155_m;
    constexpr auto L2ArmAngle                    = 45.0_deg;
    constexpr auto L2WristAngle                  = 90.0_deg;
    constexpr auto L2GripperBothWheels           = true;
    constexpr auto L2GripperVoltage              = CoralGripVoltage;

    constexpr auto L3Elevator                    = 0.789707_m;
    constexpr auto L3ArmAngle                    = 45.0_deg;
    constexpr auto L3WristAngle                  = 90.0_deg;
    constexpr auto L3GripperBothWheels           = true;
    constexpr auto L3GripperVoltage              = CoralGripVoltage;

    constexpr auto L4Elevator                    = 1.68_m;
    constexpr auto L4ArmAngle                    = 93.579102_deg;
    constexpr auto L4WristAngle                  = 90.0_deg;
    constexpr auto L4GripperBothWheels           = true;
    constexpr auto L4GripperVoltage              = CoralGripVoltage;
}
#pragma endregion
