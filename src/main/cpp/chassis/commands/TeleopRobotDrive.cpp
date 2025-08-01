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

#include "chassis/commands/TeleopRobotDrive.h"
#include "utils/logging/debug/Logger.h"

// Note the simplified constructor and AddRequirements call
TeleopRobotDrive::TeleopRobotDrive(subsystems::CommandSwerveDrivetrain *chassis,
                                   TeleopControl *controller,
                                   units::velocity::meters_per_second_t maxSpeed,
                                   units::angular_velocity::degrees_per_second_t maxAngularRate) : m_chassis(chassis),
                                                                                                   m_controller(controller),
                                                                                                   m_maxSpeed(maxSpeed),
                                                                                                   m_maxAngularRate(maxAngularRate)
{
    AddRequirements(m_chassis);
}

void TeleopRobotDrive::Execute()
{
    double forward = m_controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_FORWARD);
    double strafe = m_controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_STRAFE);
    double rotate = m_controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_ROTATE);

    m_chassis->SetControl(
        m_RobotDriveRequest.WithVelocityX(forward * m_maxSpeed)
            .WithVelocityY(strafe * m_maxSpeed)
            .WithRotationalRate(rotate * m_maxAngularRate));
}

bool TeleopRobotDrive::IsFinished()
{
    // A default drive command should never finish on its own.
    // It runs until it is interrupted by another command.
    return false;
}

void TeleopRobotDrive::End(bool interrupted)
{
    m_chassis->SetControl(swerve::requests::SwerveDriveBrake{});
}