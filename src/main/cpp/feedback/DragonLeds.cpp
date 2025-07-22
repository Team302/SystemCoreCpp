
//====================================================================================================================================================
// Copyright 2025 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#include <span>
#include <string>

#include "feedback/DragonLeds.h"
#include "utils/logging/debug/Logger.h"

DragonLeds::DragonLeds() : m_addressibleLeds()
{
}

DragonLeds *DragonLeds::m_instance = nullptr;
DragonLeds *DragonLeds::GetInstance()
{
    if (m_instance == nullptr)
    {
        m_instance = new DragonLeds();
    }
    return m_instance;
}

void DragonLeds::Initialize(int PWMport, int numLeds)
{
    if (!IsInitialized())
    {
        m_addressibleLeds = new frc::AddressableLED(PWMport);
        m_addressibleLeds->SetLength(numLeds);

        m_ledBuffer.resize(numLeds);

        SetSolidColor(frc::Color::kDarkGreen);
        commitLedData();
        setOn();
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, std::string("DragonLeds"), std::string("Already defined"), std::string("Only one allowed"));
    }
}

bool DragonLeds::IsInitialized() const
{
    return m_addressibleLeds != nullptr;
}

void DragonLeds::setOn()
{
    if (IsInitialized())
    {
        m_addressibleLeds->Start();
    }
}

void DragonLeds::setOff()
{
    if (IsInitialized())
    {
        m_addressibleLeds->Stop();
    }
}
void DragonLeds::ResetVariables()
{
    m_loopThroughIndividualLEDs = -1;
    m_colorLoop = 0;
    m_timer = 0;
    m_switchColor = false;
}

void DragonLeds::commitLedData()
{
    if (m_ledBuffer.size() > 0 && IsInitialized())
    {
        m_addressibleLeds->SetData(m_ledBuffer);
    }
}
void DragonLeds::SetSolidColor(frc::Color color)
{
    if (IsInitialized())
    {
        frc::LEDPattern pattern = frc::LEDPattern::Solid(color);
        pattern.ApplyTo(m_ledBuffer);
    }
}

void DragonLeds::SetAlternatingColor(frc::Color color1, frc::Color color2)
{
    if (IsInitialized())
    {
        for (unsigned int i = 0; i < m_ledBuffer.size(); i++)
        {
            if (i % 2 == 0)
                m_ledBuffer[i].SetLED(color1);
            else
                m_ledBuffer[i].SetLED(color2);
        }
    }
}

void DragonLeds::SetScorllingRainbow()
{
    if (IsInitialized())
    {
        units::meter_t kLedSpacing{1 / 120.0};
        frc::LEDPattern m_rainbow = frc::LEDPattern::Rainbow(255, 128);
        frc::LEDPattern m_scrollingRainbow = m_rainbow.ScrollAtAbsoluteSpeed(0.25_mps, kLedSpacing);
        m_scrollingRainbow.ApplyTo(m_ledBuffer);
    }
}

void DragonLeds::SetSpecificLED(int id, frc::Color color)
{
    if (IsInitialized())
    {
        m_ledBuffer[id].SetLED(color);
    }
}

void DragonLeds::SetBufferAllLEDsBlack()
{
    if (IsInitialized())
    {
        frc::LEDPattern pattern = frc::LEDPattern::Solid(frc::Color::kBlack);
        pattern.ApplyTo(m_ledBuffer);
    }
}

void DragonLeds::SetBufferAllLEDsColorBrightness(frc::Color color, double brightness)
{
    if (IsInitialized())
    {
        frc::LEDPattern base = frc::LEDPattern::Solid(color);
        frc::LEDPattern pattern = base.AtBrightness(brightness);

        pattern.ApplyTo(m_ledBuffer);
    }
}
void DragonLeds::SetBreathingPattern(frc::Color color, units::time::second_t period)
{
    if (IsInitialized())
    {

        frc::LEDPattern base = frc::LEDPattern::Solid(color);
        frc::LEDPattern pattern = base.Breathe(period);

        pattern.ApplyTo(m_ledBuffer);
    }
}

void DragonLeds::SetBlinkingPattern(frc::Color color, units::time::second_t cycleTime)
{
    if (IsInitialized())
    {
        frc::LEDPattern base = frc::LEDPattern::Solid(color);
        frc::LEDPattern pattern = base.Blink(cycleTime);

        pattern.ApplyTo(m_ledBuffer);
    }
}

void DragonLeds::SetAlternatingColorBlinkingPattern(frc::Color c1, frc::Color c2)
{
    if (IsInitialized())
    {
        if (m_ledBuffer.size() > 0)
        {
            if (m_timer > 2 * m_blinkPatternPeriod)
                m_timer = 0;

            int blinkState = (m_timer / m_blinkPatternPeriod) % 2;

            if (blinkState == 0)
                SetAlternatingColor(c1, c2);
            else
                SetAlternatingColor(c2, c1);

            m_timer++;
        }
    }
}

void DragonLeds::SetChaserPattern(frc::Color c)
{
    if (IsInitialized())
    {
        if (m_ledBuffer.size() > 0)
        {
            m_loopThroughIndividualLEDs += m_loopThroughIndividualLEDs < static_cast<int>(m_ledBuffer.size()) - 1 ? 1 : -m_loopThroughIndividualLEDs;
            if (!m_switchColor)
            {
                m_lastColor = m_lastColor == c ? frc::Color::kBlack : c;
            }
            m_switchColor = m_loopThroughIndividualLEDs != static_cast<int>(m_ledBuffer.size()) - 1;

            SetSpecificLED(m_loopThroughIndividualLEDs, m_lastColor);
        }
    }
}

void DragonLeds::SetClosingInChaserPattern(frc::Color c)
{
    if (IsInitialized())
    {
        if (m_ledBuffer.size() > 0)
        {
            if (m_timer == 7)
            {
                int halfLength = (m_ledBuffer.size() - 1) / 2;
                m_loopThroughIndividualLEDs += m_loopThroughIndividualLEDs < halfLength ? 1 : -m_loopThroughIndividualLEDs;
                int loopout = (m_ledBuffer.size() - 1) - m_loopThroughIndividualLEDs;
                auto color = m_colorLoop >= 0 ? c : frc::Color::kBlack;
                m_colorLoop += m_colorLoop < halfLength ? 1 : -((m_colorLoop * 2) + 1);
                SetSpecificLED(m_loopThroughIndividualLEDs, color);
                SetSpecificLED(loopout, color);

                m_timer = 0;
            }
            m_timer++;
        }
    }
}

void DragonLeds::DiagnosticPattern(frc::DriverStation::Alliance alliance, bool coralInSensor, bool coralOutSensor, bool algaeSensor, bool questStatus, bool ll1Status)
{
    if (IsInitialized())
    {
        auto allianceColor = alliance == frc::DriverStation::Alliance::kBlue ? frc::Color::kBlue : frc::Color::kRed;
        SetSpecificLED(m_allianceColorLED, allianceColor);

        auto coralInSensorcolor = coralInSensor ? frc::Color::kYellow : frc::Color::kBlack;
        SetSpecificLED(m_coralInSensorDiagnosticLED, coralInSensorcolor);

        auto coralOutSensorcolor = coralOutSensor ? frc::Color::kYellow : frc::Color::kBlack;
        SetSpecificLED(m_coralOutSensorDiagnosticLED, coralOutSensorcolor);

        auto algaeSensorcolor = algaeSensor ? frc::Color::kYellow : frc::Color::kBlack;
        SetSpecificLED(m_algaeSensorDiagnosticLED, algaeSensorcolor);

        auto questStatuscolor = questStatus ? frc::Color::kGreen : frc::Color::kDarkRed;
        SetSpecificLED(m_questDiagnosticLED, questStatuscolor);

        auto ll1Statuscolor = ll1Status ? frc::Color::kGreen : frc::Color::kDarkRed;
        SetSpecificLED(m_limeLight1diagnosticLED, ll1Statuscolor);
    }
}