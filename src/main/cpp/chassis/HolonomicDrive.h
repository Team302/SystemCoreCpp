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

// C++ Libraries

#include "frc/geometry/Pose2d.h"

// Team 302 includes
#include "chassis/ChassisMovement.h"
#include "state/State.h"
#include "state/IRobotStateChangeSubscriber.h"
#include "fielddata/BargeHelper.h"
#include "fielddata/ReefHelper.h"

class SwerveChassis;

class HolonomicDrive : public State, IRobotStateChangeSubscriber
{
public:
    HolonomicDrive();
    ~HolonomicDrive() = default;

    void Init() override;
    void Run() override;
    void Exit() override;
    bool AtTarget() override;

private:
    void InitChassisMovement();
    void InitSpeeds(double forwardScale, double strafeScale, double rotateScale);
    void ResetYaw();
    void AlignGamePiece();
    void HoldPosition();
    void TurnForward();
    void TurnBackward();
    void SlowMode();
    void CheckTipping(bool tippingSelected);
    void CheckRobotOriented(bool robotOrientedSelected);
    void PolarDrive();
    void DriveToFieldElement(double forward, double strafe, double rot, ChassisOptionEnums::DriveStateType driveState);
    void DriveToGamePiece(double forward, double strafe, double rot);
    void NotifyStateUpdate(RobotStateChanges::StateChange change, units::length::meter_t value) override;
    void NotifyStateUpdate(RobotStateChanges::StateChange change, int value) override;

    SwerveChassis *m_swerve;
    ChassisOptionEnums::DriveStateType m_previousDriveState;
    const double m_slowModeMultiplier = 0.25;
    const double m_inputScale = 1.0;
    bool m_CheckTipping = false;
    bool m_checkTippingLatch = false;
    ChassisMovement m_moveInfo;

    bool m_robotOrientedLatch = false;
    bool m_robotOrientedDrive = false;
    bool m_resetPathplannerTrajectory = false;
    units::length::inch_t m_elevatorHeight;
    units::length::inch_t m_elevatorHeightThreshold = units::length::inch_t(20.0);
    double m_dynamicSpeed = 1.0;
    bool m_climbMode = false;
    bool m_mlPipeline = false;
    ReefHelper *m_reefHelper;
    BargeHelper *m_bargeHelper;
};