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

#include "chassis/commands/VisionDrive.h"

// Note the simplified constructor and AddRequirements call
VisionDrive::VisionDrive(subsystems::CommandSwerveDrivetrain *chassis,
                         TeleopControl *controller,
                         units::velocity::meters_per_second_t maxSpeed,
                         units::angular_velocity::degrees_per_second_t maxAngularRate) : m_chassis(chassis),
                                                                                         m_controller(controller),
                                                                                         m_maxSpeed(maxSpeed),
                                                                                         m_maxAngularRate(maxAngularRate)
{
    AddRequirements(m_chassis);
    m_drivePID.SetIZone(5.0);
    m_rotatePID.SetIZone(5.0);
}

void VisionDrive::Initialize()
{
    if (m_vision != nullptr)
    {
        m_vision->SetPipeline(DRAGON_LIMELIGHT_CAMERA_USAGE::ALGAE_AND_APRIL_TAGS, DRAGON_LIMELIGHT_PIPELINE::MACHINE_LEARNING_PL);
    }
}

void VisionDrive::Execute()
{

    bool hasTarget = m_vision->HasTarget(DRAGON_LIMELIGHT_CAMERA_USAGE::ALGAE_AND_APRIL_TAGS);
    if (hasTarget)
    {
        auto tx = m_vision->GetTx(DRAGON_LIMELIGHT_CAMERA_USAGE::ALGAE_AND_APRIL_TAGS);
        auto ty = -m_vision->GetTy(DRAGON_LIMELIGHT_CAMERA_USAGE::ALGAE_AND_APRIL_TAGS);

        auto rotate = std::clamp(units::angular_velocity::degrees_per_second_t(m_rotatePID.Calculate(tx.value())), -m_visionAngularRate, m_visionAngularRate);
        auto forward = std::clamp(units::velocity::meters_per_second_t(m_drivePID.Calculate(ty.value())), -m_maxVisionSpeed, m_maxVisionSpeed);

        m_chassis->SetControl(
            m_RobotDriveRequest.WithVelocityX(forward)
                .WithVelocityY(0_mps)
                .WithRotationalRate(rotate));
    }
    else
    {
        double forward = m_controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_FORWARD);
        double strafe = m_controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_STRAFE);
        double rotate = m_controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_ROTATE);

        m_chassis->SetControl(
            m_fieldDriveRequest.WithVelocityX(forward * m_maxSpeed)
                .WithVelocityY(strafe * m_maxSpeed)
                .WithRotationalRate(rotate * m_maxAngularRate));
    }
}

bool VisionDrive::IsFinished()
{
    // A default drive command should never finish on its own.
    // It runs until it is interrupted by another command.
    return false;
}

void VisionDrive::End(bool interrupted)
{
    m_chassis->SetControl(swerve::requests::SwerveDriveBrake{});
}