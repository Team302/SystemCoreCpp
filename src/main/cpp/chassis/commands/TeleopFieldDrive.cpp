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

#include "chassis/commands/TeleopFieldDrive.h"
#include "state/RobotState.h"
#include "utils/FMSData.h"

// Note the simplified constructor and AddRequirements call
TeleopFieldDrive::TeleopFieldDrive(subsystems::CommandSwerveDrivetrain *chassis,
                                   TeleopControl *controller,
                                   units::velocity::meters_per_second_t maxSpeed,
                                   units::angular_velocity::degrees_per_second_t maxAngularRate) : m_chassis(chassis),
                                                                                                   m_controller(controller),
                                                                                                   m_maxSpeed(maxSpeed),
                                                                                                   m_maxAngularRate(maxAngularRate)
{
    AddRequirements(m_chassis);
    m_targetFinder = DragonTargetFinder::GetInstance();
    m_fieldHeadingDriveRequest.WithHeadingPID(m_heading_kP, m_heading_kI, m_heading_kD);
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::ClimbModeStatus_Int);
}

void TeleopFieldDrive::Initialize()
{
    RobotState::GetInstance()->PublishStateChange(RobotStateChanges::DriveToFieldElementIsDone_Bool, false);
    RobotState::GetInstance()->PublishStateChange(RobotStateChanges::StateChange::IsInBargeZone_Bool, false);
    RobotState::GetInstance()->PublishStateChange(RobotStateChanges::StateChange::IsInReefZone_Bool, false);

    auto vision = DragonVision::GetDragonVision();
    if (vision != nullptr)
    {
        vision->SetPipeline(DRAGON_LIMELIGHT_CAMERA_USAGE::ALGAE_AND_APRIL_TAGS, DRAGON_LIMELIGHT_PIPELINE::APRIL_TAG);
    }
}

void TeleopFieldDrive::Execute()
{
    double forward = m_controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_FORWARD);
    double strafe = m_controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_STRAFE);
    double rotate = m_controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_ROTATE);

    auto isFaceReefSelected = m_controller->IsButtonPressed(TeleopControlFunctions::FACE_REEF);

    // Heading Control
    if (isFaceReefSelected)
    {
        if (m_climbMode == RobotStateChanges::ClimbMode::ClimbModeOn)
        {
            m_targetHeading = units::angle::degree_t(-90);
        }
        else
        {
            FaceReef();
        }
        m_chassis->SetControl(m_fieldHeadingDriveRequest.WithVelocityX(forward * m_maxSpeed)
                                  .WithVelocityY(strafe * m_maxSpeed)
                                  .WithTargetDirection(m_targetHeading));
    }
    else // if nothing is selected then just drive with the current heading
    {
        m_chassis->SetControl(
            m_fieldDriveRequest.WithVelocityX(forward * m_maxSpeed)
                .WithVelocityY(strafe * m_maxSpeed)
                .WithRotationalRate(rotate * m_maxAngularRate));
    }
}

bool TeleopFieldDrive::IsFinished()
{
    // A default drive command should never finish on its own.
    // It runs until it is interrupted by another command.
    return false;
}

void TeleopFieldDrive::End(bool interrupted)
{
    m_chassis->SetControl(swerve::requests::SwerveDriveBrake{});
}

void TeleopFieldDrive::FaceReef()
{
    auto info = m_targetFinder->GetPose(DragonTargetFinderTarget::CLOSEST_REEF_ALGAE);
    if (info.has_value())
    {
        auto targetpose = get<1>(info.value());
        m_targetHeading = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ? targetpose.Rotation().Degrees() - 180_deg : targetpose.Rotation().Degrees();
    }
}

void TeleopFieldDrive::NotifyStateUpdate(RobotStateChanges::StateChange change, int value)
{
    if (RobotStateChanges::StateChange::ClimbModeStatus_Int == change)
        m_climbMode = static_cast<RobotStateChanges::ClimbMode>(value);
}