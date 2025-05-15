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

#include "subsystems/Gripper.h"
#pragma endregion

#pragma region ConstantsChassisAprilTagToPose
namespace ConstantsChassisAprilTagToPose
{
    constexpr auto ChassisSpeed                     =  1.0_mps;
    constexpr auto TimeoutTime                      =  15_s;

    //*************************** Coral Station *************************************
    constexpr auto CoralStationDistanceOffsetX      =  0.0_m;
    constexpr auto CoralStationDistanceOffsetY      =  0.0_m;
    constexpr auto CoralStationAngleOffset          =  0.0_deg;

    //*************************** Reef **********************************************
    // Coral L1 right
    constexpr auto CoralL1ReefRightDistanceOffsetX  =  20.0_in;
    constexpr auto CoralL1ReefRightDistanceOffsetY  = -10.0_in;
    constexpr auto CoralL1ReefRightAngleOffset      =  0.0_deg;

    // Coral L1 left
    constexpr auto CoralL1ReefLeftDistanceOffsetX   =  18.0_in;
    constexpr auto CoralL1ReefLeftDistanceOffsetY   =  10.0_in;
    constexpr auto CoralL1ReefLeftAngleOffset       =   0.0_deg;

    // Coral L2 and L3 right
    constexpr auto CoralL23ReefRightDistanceOffsetX =  13.5_in;
    constexpr auto CoralL23ReefRightDistanceOffsetY =  -3.0_in;
    constexpr auto CoralL23ReefRightAngleOffset     =   0.0_deg;

    // Coral L2 and L3 left
    constexpr auto CoralL23ReefLeftDistanceOffsetX  =  13.5_in;
    constexpr auto CoralL23ReefLeftDistanceOffsetY  =   9.5_in;
    constexpr auto CoralL23ReefLeftAngleOffset      =   0.0_deg;

    // Coral L4 right
    constexpr auto CoralL4ReefRightDistanceOffsetX  =  15.5_in + 1.0_in;
    constexpr auto CoralL4ReefRightDistanceOffsetY  =  -3.5_in + 0.5_in;
    constexpr auto CoralL4ReefRightAngleOffset      =  0.0_deg;

    // Coral L4 left
    constexpr auto CoralL4ReefLeftDistanceOffsetX   =  15.5_in + 1.0_in;
    constexpr auto CoralL4ReefLeftDistanceOffsetY   =   8.0_in;
    constexpr auto CoralL4ReefLeftAngleOffset       =   0.0_deg;

    // Reef Algae
    constexpr auto AlgaeReefDistanceOffsetX         =  17.0_in;
    constexpr auto AlgaeReefDistanceOffsetY         =   0.0_m;
    constexpr auto AlgaeReefAngleOffset             =   0.0_deg;

    //*************************** Algae Processor ***********************************
    constexpr auto AlgaeProcessorDistanceOffsetX    =   0.0_m;
    constexpr auto AlgaeProcessorDistanceOffsetY    =   0.0_m;
    constexpr auto AlgaeProcessorAngleOffset        =   0.0_deg;

    //*************************** Algae Barge ***************************************
    constexpr auto AlgaelBargeDistanceOffsetX       =   0.0_m;
    constexpr auto AlgaelBargeDistanceOffsetY       =   0.0_m;
    constexpr auto AlgaelBargeAngleOffset           =   0.0_deg;
}
#pragma endregion
