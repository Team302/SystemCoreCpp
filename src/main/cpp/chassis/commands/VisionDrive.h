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
#include "vision/DragonVision.h"
#include "teleopcontrol/TeleopControl.h"
#include "units/velocity.h"
#include "units/angular_velocity.h"
#include "frc/controller/PIDController.h"

class VisionDrive : public frc2::CommandHelper<frc2::Command, VisionDrive>
{
public:
    VisionDrive(subsystems::CommandSwerveDrivetrain *chassis,
                TeleopControl *controller,
                units::velocity::meters_per_second_t maxSpeed,
                units::angular_velocity::degrees_per_second_t maxAngularRate);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

private:
    subsystems::CommandSwerveDrivetrain *m_chassis;
    TeleopControl *m_controller;
    units::velocity::meters_per_second_t m_maxSpeed;
    units::velocity::meters_per_second_t m_maxVisionSpeed = 1.25_mps;
    units::angular_velocity::degrees_per_second_t m_visionAngularRate = 360_deg_per_s;
    units::angular_velocity::degrees_per_second_t m_maxAngularRate;

    DragonVision *m_vision = DragonVision::GetDragonVision();

    double m_forwardkP = 0.12;
    double m_forwardkI = 0.1;
    double m_forwardkD = 0.0;
    double m_rotationkP = 5.0;
    double m_rotationkI = 1.5;
    double m_rotationkD = 0.0;

    frc::PIDController m_drivePID{m_forwardkP, m_forwardkI, m_forwardkD};
    frc::PIDController m_rotatePID{m_rotationkP, m_rotationkI, m_rotationkD};

    swerve::requests::RobotCentric m_RobotDriveRequest = swerve::requests::RobotCentric{}
                                                             .WithDeadband(m_maxSpeed * 0.1)
                                                             .WithRotationalDeadband(m_maxAngularRate * 0.1) // Add a 10% deadband
                                                             .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage)
                                                             .WithDesaturateWheelSpeeds(true); // Use open-loop control for drive motors

    swerve::requests::FieldCentric m_fieldDriveRequest = swerve::requests::FieldCentric{}
                                                             .WithDeadband(m_maxSpeed * 0.1)                                  // TODO: Investigate this deadband vs controller deadband
                                                             .WithRotationalDeadband(m_maxAngularRate * 0.1)                  // TODO: Investigate this deadband vs controller deadband
                                                             .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage) // Use open-loop voltage for drive
                                                             .WithDesaturateWheelSpeeds(true);
};