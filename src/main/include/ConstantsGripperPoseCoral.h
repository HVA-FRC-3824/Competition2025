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

#pragma region CoralPoseConstants
namespace CoralPoseConstants
{
    constexpr auto CoralGroundPickupVoltage      = 4.0_V;  // TODO: Set the voltage
    constexpr auto CoralGripVoltage              = 0.5_V;  // TODO: Set the voltage

    constexpr auto HomeElevator                  = 0.05_m;
    constexpr auto HomeArmAngle                  = 25_deg;
    constexpr auto HomeWristAngle                = 0.0_deg;
    constexpr auto HomeGripperBothWheels         = true;
    constexpr auto HomeGripperVoltage            = CoralGripVoltage;

    constexpr auto GroundElevator                = 0.164432_m;
    constexpr auto GroundArmAngle                = 138.765_deg;
    constexpr auto GroundWristAngle              = 180.0_deg;
    constexpr auto GroundGripperBothWheels       = false;
    constexpr auto GroundGripperVoltage          = CoralGroundPickupVoltage;

    constexpr auto StationElevator               = 0.5_m;
    constexpr auto StationArmAngle               = 45.0_deg;
    constexpr auto StationWristAngle             = 180.0_deg;
    constexpr auto StationGripperBothWheels      = true;
    constexpr auto StationGripperVoltage         = CoralGripVoltage;  // Verify the voltage

    constexpr auto L1Elevator                    = 0.347767_m;
    constexpr auto L1ArmAngle                    = 80.0_deg;
    constexpr auto L1WristAngle                  = 180.0_deg;
    constexpr auto L1GripperBothWheels           = true;
    constexpr auto L1GripperVoltage              = CoralGripVoltage;

    constexpr auto L2Elevator                    = 0.452952_m;
    constexpr auto L2ArmAngle                    = 45.0_deg;
    constexpr auto L2WristAngle                  = 90.0_deg;
    constexpr auto L2GripperBothWheels           = true;
    constexpr auto L2GripperVoltage              = CoralGripVoltage;

    constexpr auto L3Elevator                    = 0.801765_m;
    constexpr auto L3ArmAngle                    = 45.0_deg;
    constexpr auto L3WristAngle                  = 90.0_deg;
    constexpr auto L3GripperBothWheels           = true;
    constexpr auto L3GripperVoltage              = CoralGripVoltage;

    constexpr auto L4Elevator                    = 1.659898_m;
    constexpr auto L4ArmAngle                    = 73.0_deg;
    constexpr auto L4WristAngle                  = 90.0_deg;
    constexpr auto L4GripperBothWheels           = true;
    constexpr auto L4GripperVoltage              = CoralGripVoltage;

    constexpr auto AutonomousL1Elevator          = L1Elevator;
    constexpr auto AutonomousL1ArmAngle          = L1ArmAngle;
    constexpr auto AutonomousL1WristAngle        = 0.0_deg;
    constexpr auto AutonomousL1GripperBothWheels = true;
    constexpr auto AutonomousL1GripperVoltage    = CoralGripVoltage;
}
#pragma endregion
