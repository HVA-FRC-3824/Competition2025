#include "subsystems/Leds.h"

using namespace std;

#pragma region Leds (constructor)
/// @brief Class to support an addressable LED string.
Leds::Leds()
{
    // Length is expensive to set, so only set it once, then just update data
    m_led.SetLength(LedConstants::Length);

    // Set the default mode
    SetMode(LedMode::Off);

    // Intialize the LED data
    m_led.SetData(m_ledBuffer);

    // Start the addressable LED communications
    m_led.Start();
}
#pragma endregion

#pragma region Periodic
/// @brief This method will be called once periodically.
void Leds::Periodic()
{
    switch (m_ledMode)
    {
        case LedMode::Off:
        case LedMode::SolidGreen:
        case LedMode::SolidRed:
            break;

        case LedMode::HvaColors:
            HvaColors();
            break;

        case LedMode::Strobe:
            Strobe();
            break;

        case LedMode::ShootingAnimation:
        {
            // Apply the shootime pattern to the data buffer
            m_shooting.ApplyTo(m_ledBuffer);
            break;
        }

        case LedMode::Rainbow:
        {
            // Run the rainbow pattern and apply it to the buffer
            m_scrollingRainbow.ApplyTo(m_ledBuffer);
            break;
        }
    }

    // Set the LEDs
    m_led.SetData(m_ledBuffer);
}
#pragma endregion

#pragma region SetMode
/// @brief Setting the Led's mode to the given parameter.
/// @param ledMode mode to set the Leds.
void Leds::SetMode(LedMode ledMode)
{
    // Remember the LED mode
    m_ledMode = ledMode;

    // Set the LEDs based on the LED mode
    switch (m_ledMode)
    {
    case LedMode::Off:
        SolidColor(0, 0, 0);
        break;

    case LedMode::SolidGreen:
        SolidColor(0, LedConstants::Green, 0);
        break;

    case LedMode::SolidRed:
        SolidColor(LedConstants::Red, 0, 0);
        break;

    case LedMode::HvaColors:
        m_cycleCounter = 0;
        HvaColors();
        break;

    case LedMode::Strobe:
        m_cycleCounter = 0;
        Strobe();
        break;

    default:
        break;
    }

    // Set the LEDs
    m_led.SetData(m_ledBuffer);
}
#pragma endregion

#pragma region SolidColor
/// @brief Method to support setting the LED string to the specified solid color.
/// @param red The red component of the LED color.
/// @param green The green component of the LED color.
/// @param blue The blue component of the LED color.
void Leds::SolidColor(int red, int green, int blue)
{
    // Set the value for every pixel
    for (auto ledIndex = 0; ledIndex < LedConstants::Length; ledIndex++)
        m_ledBuffer[ledIndex].SetRGB(red * LedConstants::Brightness, green * LedConstants::Brightness, blue * LedConstants::Brightness);
}
#pragma endregion

#pragma region HvaColors
/// @brief Method to support setting the LED string to HVA alternating color.
void Leds::HvaColors()
{
    int firstColor  = LedConstants::Blue;
    int secondColor = 0;

    // Alternate the colors
    if (m_cycleCounter % LedConstants::HvaDelay < LedConstants::HvaDelay / 2)
    {
        firstColor  = 0;
        secondColor = LedConstants::Blue;
    }

    // For every pixel
    for (auto ledIndex = 0; ledIndex < LedConstants::Length; ledIndex++)
    {
        // Set the color based on the pixel index
        if (ledIndex % 2 == 0)
            m_ledBuffer[ledIndex].SetRGB(0, 0, firstColor * LedConstants::Brightness);
        else
            m_ledBuffer[ledIndex].SetRGB(0, 0, secondColor * LedConstants::Brightness);
    }

    // Update the cycle counter
    m_cycleCounter++;
}
#pragma endregion

#pragma region Strobe
/// @brief Method to strobe the LED string.
void Leds::Strobe()
{
    if (m_cycleCounter % LedConstants::StrobeDelay == 0)
        SolidColor(LedConstants::Red, LedConstants::Green, LedConstants::Blue);
    else
        SolidColor(0, 0, 0);

    // Update the cycle counter
    m_cycleCounter++;
}
#pragma endregion
