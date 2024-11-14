// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <array>

#include "../Constants.h"

#include <frc/AddressableLED.h>

/// @brief modes for the LEDs
enum LedMode 
{
  LedOff,
  SolidGreen,
  SolidRed,
  HvaColors,
  Strobe,
  ShootingAnimation,
  Rainbow
};

class Leds : public frc2::SubsystemBase
{
public:
  Leds();

  // Will be called periodically whenever the CommandScheduler runs.
  void Periodic() override;

  void SetMode(LedMode ledMode);

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  
  int m_counter = 0;

  LedMode m_ledMode;

  static constexpr int kLength = 410;

  // Must be a PWM header, not MXP or DIO
  frc::AddressableLED m_led{LED_PWM_PORT};
  std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer; // Reuse the buffer

  // Store what the last hue of the first pixel is
  int firstPixelHue = 0;
  int cycleCounter = 0;
  
  void Rainbow();
  void LedOff();
  void SolidColor(int red, int green, int blue);
  void HvaColors();
  void Strobe();
  void ShootingAnimation();
};
