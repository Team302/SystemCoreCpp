
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

#include "frc/DriverStation.h"
#include "state/RobotStateChanges.h"
#include <units/length.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <frc/geometry/Pose2d.h>

class SwerveChassis;
class IRobotStateChangeSubscriber;
class RobotStateChangeBroker;
class TeleopControl;

class RobotState
{
public:
    void Init();
    void Run();
    static RobotState *GetInstance();
    void RegisterForStateChanges(IRobotStateChangeSubscriber *subscriber, RobotStateChanges::StateChange change);
    void PublishStateChange(RobotStateChanges::StateChange change, int newValue);
    void PublishStateChange(RobotStateChanges::StateChange change, double newValue);
    void PublishStateChange(RobotStateChanges::StateChange change, units::length::meter_t newValue);
    void PublishStateChange(RobotStateChanges::StateChange change, units::angle::degree_t newValue);
    void PublishStateChange(RobotStateChanges::StateChange change, units::velocity::meters_per_second_t newValue);
    void PublishStateChange(RobotStateChanges::StateChange change, units::angular_velocity::degrees_per_second_t newValue);
    void PublishStateChange(RobotStateChanges::StateChange change, frc::Pose2d newValue);
    void PublishStateChange(RobotStateChanges::StateChange change, bool newValue);

private:
    void PublishGameStateChanges();
    void PublishScoringMode(TeleopControl *controller);
    void PublishClimbMode(TeleopControl *controller);
    void PublishDesiredCoralSide(TeleopControl *controller);

    RobotState();
    ~RobotState();

    std::vector<RobotStateChangeBroker *> m_brokers;
    RobotStateChanges::ScoringMode m_scoringMode;
    RobotStateChanges::GamePeriod m_gamePhase;
    RobotStateChanges::ClimbMode m_climbMode;
    RobotStateChanges::DesiredCoralSide m_desiredCoralSide = RobotStateChanges::DesiredCoralSide::Sidewall;

    bool m_scoringModeButtonReleased;
    bool m_climbModeButtonReleased;
    bool m_desiredCoralSideButtonReleased;
    static RobotState *m_instance;
};
