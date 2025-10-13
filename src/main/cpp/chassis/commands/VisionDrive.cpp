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
                         units::angular_velocity::degrees_per_second_t maxAngularRate,
                         DragonVision::VISION_ELEMENT visionElement,
                         units::length::inch_t xOffset,
                         units::length::inch_t yOffset) : m_chassis(chassis),
                                                          m_controller(controller),
                                                          m_maxSpeed(maxSpeed),
                                                          m_maxAngularRate(maxAngularRate),
                                                          m_visionElement(visionElement),
                                                          m_xOffset(xOffset),
                                                          m_yOffset(yOffset)
{
    AddRequirements(m_chassis);
    m_drivePID.SetIZone(5.0);
    m_drivePID.SetIntegratorRange(-5, 5.0);

    m_strafePID.SetIZone(5.0);
    m_strafePID.SetIntegratorRange(-5, 5.0);

    m_rotatePID.SetIZone(5.0);
    m_rotatePID.EnableContinuousInput(-180.0, 180.0);
    m_rotatePID.SetIntegratorRange(-30.0, 30.0);
}

void VisionDrive::Initialize()
{
    if (m_vision != nullptr)
    {
        m_vision->SetPipeline(DRAGON_LIMELIGHT_CAMERA_USAGE::OBJECT_DETECTION_ALGAE, DRAGON_LIMELIGHT_PIPELINE::MACHINE_LEARNING_PL);
    }
    m_drivePID.Reset();
    m_strafePID.Reset();
    m_rotatePID.Reset();
}

void VisionDrive::Execute()
{
    auto visionDatan = m_vision->GetVisionData(m_visionElement);

    if (visionDatan.has_value())
    {
        auto dragonTargetFinderInst = DragonTargetFinder::GetInstance();
        auto erros = dragonTargetFinderInst->CalculateTargetingErrors(visionDatan.value(), m_xOffset, m_yOffset);

        auto rotate = std::clamp(units::angular_velocity::degrees_per_second_t(m_rotatePID.Calculate(erros.value().yawError.value())), -m_visionAngularRate, m_visionAngularRate);
        auto forward = std::clamp(units::velocity::meters_per_second_t(m_drivePID.Calculate(erros.value().xError.value())), -m_maxVisionSpeed, m_maxVisionSpeed);
        auto strafe = std::clamp(units::velocity::meters_per_second_t(m_strafePID.Calculate(erros.value().yError.value())), -m_maxVisionSpeed, m_maxVisionSpeed);

        m_chassis->SetControl(
            m_RobotDriveRequest.WithVelocityX(forward)
                .WithVelocityY(strafe)
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