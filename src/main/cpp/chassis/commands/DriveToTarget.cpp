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
#include "chassis/commands/DriveToTarget.h" // Update path if needed
#include "utils/AngleUtils.h"
#include "utils/logging/debug/Logger.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Translation2d.h"
#include "vision/DragonVisionStructLogger.h"
#include "fielddata/ReefHelper.h"
#include "fielddata/BargeHelper.h"

DriveToTarget::DriveToTarget(
    subsystems::CommandSwerveDrivetrain *chassis,
    DragonTargetFinderTarget target) : m_chassis(chassis),
                                       m_target(target),
                                       m_targetFinder(DragonTargetFinder::GetInstance()),
                                       m_hasTarget(false)

{
    AddRequirements(m_chassis);
    m_translationPIDX.SetIZone(0.20);
    m_translationPIDY.SetIZone(0.20);
    m_prevPose = m_chassis != nullptr ? m_chassis->GetPose() : frc::Pose2d();
    m_feedForwardRange = m_ffMaxRadius - m_ffMinRadius;
}

void DriveToTarget::Initialize()
{
    m_chassis->ResetSamePose();
    auto dragonTargetFinderInst = DragonTargetFinder::GetInstance();
    dragonTargetFinderInst->ResetGoalPose();
    auto info = dragonTargetFinderInst->GetPose(m_target);
    m_hasTarget = false;

    if (info.has_value())
    {
        m_currentType = get<0>(info.value());
        m_hasTarget = m_currentType != DragonTargetFinderData::NOT_FOUND;
        if (m_hasTarget)
        {
            m_endPose = get<1>(info.value());

            if (m_chassis != nullptr)
            {
                m_currentPose = m_chassis->GetPose();
                m_translationPIDX.SetGoal(m_endPose.X());
                m_translationPIDY.SetGoal(m_endPose.Y());

                auto speeds = m_chassis->GetState().Speeds;

                m_translationPIDX.Reset(m_currentPose.X(), speeds.vx);
                m_translationPIDY.Reset(m_currentPose.Y(), speeds.vy);
            }
        }
    }
}

void DriveToTarget::Execute()
{
    frc::ChassisSpeeds chassisSpeeds{};
    if (m_chassis != nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToFieldElement", "target found", m_hasTarget);
        if (m_hasTarget)
        {
            m_currentPose = m_chassis->GetPose();
            CalculateFeedForward(chassisSpeeds);

            DragonVisionStructLogger::logPose2d("current pose", m_currentPose);
            DragonVisionStructLogger::logPose2d("target pose", m_endPose);

            if (m_currentType != DragonTargetFinderData::NOT_FOUND)
            {
                if (m_distanceError > m_pidResetThreshold)
                {
                    m_translationPIDX.Reset(m_currentPose.X(), chassisSpeeds.vx);
                    m_translationPIDY.Reset(m_currentPose.Y(), chassisSpeeds.vy);
                }
                else
                {
                    chassisSpeeds.vx += units::velocity::meters_per_second_t(m_translationPIDX.Calculate(m_currentPose.X(), m_endPose.X()));
                    chassisSpeeds.vy += units::velocity::meters_per_second_t(m_translationPIDY.Calculate(m_currentPose.Y(), m_endPose.Y()));
                    chassisSpeeds.vx = std::clamp(chassisSpeeds.vx, -kMaxVelocity, kMaxVelocity);
                    chassisSpeeds.vy = std::clamp(chassisSpeeds.vy, -kMaxVelocity, kMaxVelocity);
                }
            }
        }
        m_chassis->SetControl(
            m_driveRequest.WithVelocityX(chassisSpeeds.vx)
                .WithVelocityY(chassisSpeeds.vy)
                .WithTargetDirection(m_endPose.Rotation().Degrees())
                .WithHeadingPID(m_rotationKP, m_rotationKI, m_rotationKD)
                .WithForwardPerspective(ctre::phoenix6::swerve::requests::ForwardPerspectiveValue::BlueAlliance));
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToFieldElement", "Error", m_endPose.Translation().Distance(m_currentPose.Translation()).value());
    }

    if (m_target == DragonTargetFinderTarget::BARGE) // TODO: Come back and see if there is a better way to implement this including the publishing in End() method
    {
        auto bargeHelper = BargeHelper::GetInstance();
        bargeHelper->IsInZone();
    }
    else if (m_target == DragonTargetFinderTarget::CLOSEST_LEFT_REEF_BRANCH || m_target == DragonTargetFinderTarget::CLOSEST_RIGHT_REEF_BRANCH)
    {
        auto reefHelper = ReefHelper::GetInstance();
        reefHelper->IsInZone();
    }

    RobotState::GetInstance()->PublishStateChange(RobotStateChanges::DriveToFieldElementIsDone_Bool, IsFinished());
}

bool DriveToTarget::IsFinished()
{
    bool isDone = false;
    bool isSamePose = false;

    if (m_hasTarget && m_chassis != nullptr)
    {
        auto distance = m_currentPose.Translation().Distance(m_endPose.Translation());

        isDone = distance < m_distanceThreshold;
        isSamePose = m_chassis->IsSamePose();
        m_prevPose = m_currentPose;
    }
    else
    {
        isDone = true;
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToFieldElement", "Is Done", isDone);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToFieldElement", "Is SamePose", isSamePose);
    return (isDone || isSamePose);
}

void DriveToTarget::End(bool interrupted)
{
    m_chassis->SetControl(swerve::requests::SwerveDriveBrake{});
}

void DriveToTarget::CalculateFeedForward(frc::ChassisSpeeds &chassisSpeeds)
{
    if (m_chassis != nullptr)
    {
        m_distanceError = m_currentPose.Translation().Distance(m_endPose.Translation());

        units::velocity::meters_per_second_t feedforwardSpeed = 0.0_mps;
        if (m_distanceError > m_ffMinRadius)
        {
            double feedForwardScale = std::clamp(((m_distanceError - m_ffMinRadius) / (m_feedForwardRange)).value(), 0.0, 1.0);
            feedforwardSpeed = kMaxVelocity * feedForwardScale;
        }

        frc::Translation2d translationError = m_endPose.Translation() - m_currentPose.Translation();
        frc::Rotation2d angleToTarget = translationError.Angle();

        chassisSpeeds.vx = feedforwardSpeed * angleToTarget.Cos();
        chassisSpeeds.vy = feedforwardSpeed * angleToTarget.Sin();
    }
}
