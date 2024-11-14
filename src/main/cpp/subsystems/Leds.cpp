// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "../include/subsystems/Leds.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace std;

#define BRIGHTNESS 0.5
#define RAINBOW_RATE 3

Leds::Leds()
{
    frc::SmartDashboard::PutString("LEDS", "Constructor");

    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_led.SetLength(kLength);

    // m_led.SetSyncTime(1000us);
    // m_led.SetBitTiming(300ns, 800ns, 800ns, 300ns);
    SetMode(LedMode::LedOff);

    m_led.SetData(m_ledBuffer);
    m_led.Start();
}

// This method will be called once per scheduler run
void Leds::Periodic()
{
    frc::SmartDashboard::PutNumber("LED Counter: ", ++m_counter);

    frc::SmartDashboard::PutNumber("LED Mode: ", m_ledMode);

    switch (m_ledMode)
    {
    case LedMode::LedOff:
    case LedMode::SolidGreen:
    case LedMode::SolidRed:
    case LedMode::HvaColors:
        return;
    case LedMode::Strobe:
        Strobe();
        break;
    case LedMode::ShootingAnimation:
        ShootingAnimation();
        break;
    case LedMode::Rainbow:
        Rainbow();
    }


    // Set the LEDs
    m_led.SetData(m_ledBuffer);
}

/// @brief Setting the Led's mode to the given parameter.
/// @param ledMode mode to set the Leds.
void Leds::SetMode(LedMode ledMode)
{
    cout << "inside SetMode(): " << ledMode << endl;
    m_ledMode = ledMode;

    switch (m_ledMode)
    {
    case LedMode::LedOff:
        SolidColor(0, 0, 0);
        break;
    case LedMode::SolidGreen:
        SolidColor(0, 255, 0);
        break;
    case LedMode::SolidRed:
        SolidColor(255, 0, 0);
        break;
    case LedMode::HvaColors:
        cycleCounter = 0;
        HvaColors();
        break;
    case LedMode::Strobe:
        cycleCounter = 0;
        Strobe();
        break;
    case LedMode::ShootingAnimation:
        cycleCounter = 0;
        ShootingAnimation();
        break;
    case LedMode::Rainbow:
        Rainbow();
    default:
        break;
    }

    // Set the LEDs
    m_led.SetData(m_ledBuffer);
}

void Leds::Rainbow()
{
    // For every pixel
    for (int ledIndex = 0; ledIndex < kLength; ledIndex++)
    {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        const auto pixelHue = (firstPixelHue + (ledIndex * 45 / kLength)) % 360;
        // Set the value
        m_ledBuffer[ledIndex].SetHSV(pixelHue, 255, (int)(255 * BRIGHTNESS));
    }
    // Increase by to make the rainbow "move"
    firstPixelHue += RAINBOW_RATE;
    // Check bounds
    firstPixelHue %= 180;
}

void Leds::SolidColor(int red, int green, int blue)
{
    // For every pixel
    for (int ledIndex = 0; ledIndex < kLength; ledIndex++)
    {
        // Set the value
        m_ledBuffer[ledIndex].SetRGB(red * BRIGHTNESS, green * BRIGHTNESS, blue * BRIGHTNESS);
    }
}

void Leds::HvaColors()
{
    // For every pixel
    for (int ledIndex = 0; ledIndex < kLength; ledIndex++)
    {
        if (ledIndex % 2 == 0)
        {
            // Set the value
            m_ledBuffer[ledIndex].SetRGB(0, 0, 255 * BRIGHTNESS);
        }
        else
        {
            // Set the value
            m_ledBuffer[ledIndex].SetRGB(0, 0, 100 * BRIGHTNESS);
        }
    }

    cycleCounter++;
}

void Leds::Strobe()
{
    if (cycleCounter % 20 == 0)
    {
        SolidColor(255, 255, 255);
    }
    else
    {
        SolidColor(0, 0, 0);
    }

    cycleCounter++;
}

void Leds::ShootingAnimation()
{
}