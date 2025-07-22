
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

#pragma once

#include <vector>
#include <array>
#include <frc/AddressableLED.h>
#include "frc/LEDPattern.h"

#include <utils/FMSData.h>

class DragonLeds
{
public:
	std::vector<frc::AddressableLED::LEDData> m_ledBuffer;

	void Initialize(int PWMport, int numLeds);
	bool IsInitialized() const;

	void commitLedData();

	void setOn();
	void setOff();
	void ResetVariables();

	void SetSolidColor(frc::Color colar);
	void SetAlternatingColor(frc::Color colar1, frc::Color colar2);
	void SetBufferAllLEDsBlack();
	void SetScorllingRainbow();
	void SetSpecificLED(int id, frc::Color colar);
	void SetBufferAllLEDsColorBrightness(frc::Color colar, double brightness);
	void SetBreathingPattern(frc::Color colar, units::time::second_t period);
	void SetBlinkingPattern(frc::Color colar, units::time::second_t cycleTime);
	void SetAlternatingColorBlinkingPattern(frc::Color c1, frc::Color c2);
	void SetChaserPattern(frc::Color c);
	void SetClosingInChaserPattern(frc::Color c);

	void DiagnosticPattern(frc::DriverStation::Alliance alliance, bool coralInSensor, bool coralOutSensor, bool algaeSensor, bool questStatus, bool ll1Status);

	static DragonLeds *GetInstance();

private:
	static DragonLeds *m_instance;
	frc::AddressableLED *m_addressibleLeds;
	int m_numberofDiagnosticLEDs = 6;

	int m_loopThroughIndividualLEDs = -1;
	int m_colorLoop = 0;
	int m_timer = 0;
	bool m_switchColor = false;
	frc::Color m_lastColor = frc::Color::kBlack;

	const int m_blinkPatternPeriod = 10;
	const int m_allianceColorLED = 0;
	const int m_coralInSensorDiagnosticLED = 1;
	const int m_coralOutSensorDiagnosticLED = 2;
	const int m_algaeSensorDiagnosticLED = 3;
	const int m_questDiagnosticLED = 4;
	const int m_limeLight1diagnosticLED = 5;

	DragonLeds();
};
