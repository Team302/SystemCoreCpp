
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
#include <feedback/DragonLeds.h>
#include "chassis/ChassisOptionEnums.h"
#include <state/IRobotStateChangeSubscriber.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

class DriverFeedback : public IRobotStateChangeSubscriber
{
public:
    void UpdateFeedback();

    static DriverFeedback *GetInstance();

    void UpdateLEDStates();

    void UpdateCompressorState();

    void NotifyStateUpdate(RobotStateChanges::StateChange change, int value) override;
    void NotifyStateUpdate(RobotStateChanges::StateChange change, bool value) override;

private:
    void UpdateRumble();
    void UpdateDiagnosticLEDs();
    void CheckControllers();
    void DisplayPressure() const;
    void DisplayDesiredGamePiece();
    void ResetRequests(void);
    void QueryNT();
    DriverFeedback();
    ~DriverFeedback() = default;

    bool m_AutonomousEnabled = false;
    bool m_TeleopEnabled = false;

    frc::Color oldState = frc::Color::kGhostWhite;
    frc::Color currentState = frc::Color::kBlack;

    enum DriverFeedbackStates
    {
        NONE
    };

    DragonLeds *m_LEDStates = DragonLeds::GetInstance();
    int m_controllerCounter = 0;
    int m_rumbleLoopCounter = 0;
    int m_firstloop = true;

    units::time::second_t m_breathingPeriod{1.5};
    units::time::millisecond_t m_blinkingPeriod{100};

    static DriverFeedback *m_instance;
    RobotStateChanges::ScoringMode m_scoringMode = RobotStateChanges::ScoringMode::Coral;
    RobotStateChanges::ClimbMode m_climbMode = RobotStateChanges::ClimbMode::ClimbModeOff;
    bool m_driveToIsDone = false;
    ChassisOptionEnums::DriveStateType m_driveStateType = ChassisOptionEnums::DriveStateType::ROBOT_DRIVE;
};
