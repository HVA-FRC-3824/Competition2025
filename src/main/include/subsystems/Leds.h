#pragma once

#include "Constants.h"

#include <array>

#include <frc2/command/SubsystemBase.h>
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
