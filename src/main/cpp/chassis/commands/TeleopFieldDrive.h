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

#include "frc2/command/CommandHelper.h"
#include "frc2/command/Command.h"
#include "chassis/generated/CommandSwerveDrivetrain.h"
#include "teleopcontrol/TeleopControl.h"
#include "fielddata/DragonTargetFinder.h"
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include "state/IRobotStateChangeSubscriber.h"

class TeleopFieldDrive : public frc2::CommandHelper<frc2::Command, TeleopFieldDrive>, public IRobotStateChangeSubscriber
{
public:
    TeleopFieldDrive(subsystems::CommandSwerveDrivetrain *chassis,
                     TeleopControl *controller,
                     units::velocity::meters_per_second_t maxSpeed,
                     units::angular_velocity::degrees_per_second_t maxAngularRate);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;
    void NotifyStateUpdate(RobotStateChanges::StateChange change, int value) override;

private:
    subsystems::CommandSwerveDrivetrain *m_chassis;
    TeleopControl *m_controller;
    units::velocity::meters_per_second_t m_maxSpeed;
    units::angular_velocity::degrees_per_second_t m_maxAngularRate;
    DragonTargetFinder *m_targetFinder;
    units::angle::degree_t m_targetHeading{0_deg};
    static constexpr double m_heading_kP{10.0};
    static constexpr double m_heading_kI{1.0};
    static constexpr double m_heading_kD{0.0};

    RobotStateChanges::ClimbMode m_climbMode = RobotStateChanges::ClimbMode::ClimbModeOff;

    swerve::requests::FieldCentric m_fieldDriveRequest = swerve::requests::FieldCentric{}
                                                             .WithDeadband(m_maxSpeed * 0.1)                                  // TODO: Investigate this deadband vs controller deadband
                                                             .WithRotationalDeadband(m_maxAngularRate * 0.1)                  // TODO: Investigate this deadband vs controller deadband
                                                             .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage) // Use open-loop voltage for drive
                                                             .WithDesaturateWheelSpeeds(true);

    swerve::requests::FieldCentricFacingAngle m_fieldHeadingDriveRequest = swerve::requests::FieldCentricFacingAngle{}
                                                                               .WithDeadband(m_maxSpeed * 0.1)                                  // TODO: Investigate this deadband vs controller deadband
                                                                               .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage) // Use open-loop voltage for drive
                                                                               .WithDesaturateWheelSpeeds(true);

    void FaceReef();
    void FaceBarge();
};