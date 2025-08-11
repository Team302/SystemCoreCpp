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
//=====================================================================================================================================================

// C++ Includes

// FRC Includes
#include <frc/geometry/Pose2d.h>
#include <pathplanner/lib/path/PathConstraints.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <frc/smartdashboard/SmartDashboard.h>

// Team302 Includes
#include "chassis/definitions/ChassisConfig.h"
#include "chassis/definitions/ChassisConfigMgr.h"
#include "chassis/states/DriveToFieldElement.h"
#include "chassis/SwerveChassis.h"
#include "vision/DragonVisionStructs.h"
#include "vision/DragonVisionStructLogger.h"
#include "fielddata/DragonTargetFinder.h"
#include "utils/AngleUtils.h"
#include "state/RobotState.h"
#include "fielddata/ProcessorHelper.h"

#include "utils/logging/debug/Logger.h"
#include "utils/logging/debug/LoggerData.h"
#include "utils/logging/debug/LoggerEnums.h"
#include "chassis/LogChassisMovement.h"

using namespace pathplanner;
using namespace std;

DriveToFieldElement::DriveToFieldElement(RobotDrive *robotDrive) : RobotDrive(robotDrive->GetChassis()),
                                                                   m_robotDrive(robotDrive)
{
    m_translationPIDX.SetIZone(0.10);
    m_translationPIDY.SetIZone(0.10);
    m_prevPose = m_chassis != nullptr ? m_chassis->GetPose() : frc::Pose2d();
    m_feedForwardRange = m_ffMaxRadius - m_ffMinRadius;
#ifdef SHUFFLEBOARD_PIDS
    frc::SmartDashboard::PutNumber(m_iGainKey, m_translationKI);
    frc::SmartDashboard::PutNumber(m_pGainKey, m_translationKP);
#endif
}

void DriveToFieldElement::Init(ChassisMovement &chassisMovement)
{
    auto dragonTargetFinderInst = DragonTargetFinder::GetInstance();
    dragonTargetFinderInst->ResetGoalPose();
    auto info = dragonTargetFinderInst->GetPose(GetDriveToTarget());

    m_hasTarget = false;

#ifdef SHUFFLEBOARD_PIDS
    auto pGainShuffleboard = frc::SmartDashboard::GetNumber(m_pGainKey, 0);
    auto iGainShuffleboard = frc::SmartDashboard::GetNumber(m_iGainKey, 0);

    if (pGainShuffleboard != 0 && iGainShuffleboard != 0)
    {
        m_translationKP = pGainShuffleboard;
        m_translationKI = iGainShuffleboard;
    }
#endif

    if (info.has_value())
    {
        InitChassisMovement(chassisMovement);

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

                m_translationPIDX.Reset(m_currentPose.X(), chassisMovement.chassisSpeeds.vx);
                m_translationPIDY.Reset(m_currentPose.Y(), chassisMovement.chassisSpeeds.vy);
            }
        }
    }
}

std::array<frc::SwerveModuleState, 4> DriveToFieldElement::UpdateSwerveModuleStates(ChassisMovement &chassisMovement)
{
    if (m_chassis != nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToFieldElement", "target found", m_hasTarget);
        if (m_hasTarget)
        {
            m_currentPose = m_chassis->GetPose();
            CalculateFeedForward(chassisMovement);
            auto chassisSpeeds = chassisMovement.chassisSpeeds;

            auto info = DragonTargetFinder::GetInstance()->GetPose(GetDriveToTarget());
            if (info.has_value())
            {
                frc::Pose2d newEndPose = get<1>(info.value());
                auto regenerate = false;

                regenerate = m_endPose.Translation().Distance(newEndPose.Translation()) < m_regenerationDistanceThreshold;

                if ((get<0>(info.value()) == DragonTargetFinderData::VISION_BASED) && regenerate) // If we are in odometry but get vision based pose regenerate
                {
                    m_endPose = newEndPose;
                    m_translationPIDX.SetGoal(m_endPose.X());
                    m_translationPIDY.SetGoal(m_endPose.Y());
                }
                m_currentType = get<0>(info.value());
            }

            DragonVisionStructLogger::logPose2d("current pose", m_currentPose);
            DragonVisionStructLogger::logPose2d("target pose", m_endPose);

            if (m_currentType != DragonTargetFinderData::NOT_FOUND)
            {
                if (m_distanceError > m_pidResetThreshold)
                {
                    m_translationPIDX.Reset(m_currentPose.X(), chassisMovement.chassisSpeeds.vx);
                    m_translationPIDY.Reset(m_currentPose.Y(), chassisMovement.chassisSpeeds.vy);
                }
                else
                {
                    chassisSpeeds.vx += units::velocity::meters_per_second_t(m_translationPIDX.Calculate(m_currentPose.X(), m_endPose.X()));
                    chassisSpeeds.vy += units::velocity::meters_per_second_t(m_translationPIDY.Calculate(m_currentPose.Y(), m_endPose.Y()));
                    chassisSpeeds.vx = std::clamp(chassisSpeeds.vx, -kMaxVelocity, kMaxVelocity);
                    chassisSpeeds.vy = std::clamp(chassisSpeeds.vy, -kMaxVelocity, kMaxVelocity);
                }

                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("DriveToFieldElement"), "Vx", chassisSpeeds.vx.value());
                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("DriveToFieldElement"), "Vy", chassisSpeeds.vy.value());

                if (TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::SWEEP))
                {
                    chassisMovement.yawAngle = (chassisMovement.driveOption == ChassisOptionEnums::DriveStateType::DRIVE_TO_LEFT_REEF_BRANCH) ? chassisMovement.yawAngle + m_sweepDelta : chassisMovement.yawAngle - m_sweepDelta;
                }
                else
                {
                    auto rotationError = GetDriveStateType() != ChassisOptionEnums::DriveStateType::DRIVE_TO_ALGAE ? chassisMovement.yawAngle - m_currentPose.Rotation().Degrees() : m_endPose.Rotation().Degrees() - m_currentPose.Rotation().Degrees();
                    rotationError = AngleUtils::GetEquivAngle(rotationError);
                    chassisSpeeds.omega = std::clamp(units::angular_velocity::degrees_per_second_t(m_rotationKP * rotationError.value()), -kMaxAngularVelocity, kMaxAngularVelocity);
                }

                auto rot2d = frc::Rotation2d(m_chassis->GetYaw());
                chassisMovement.chassisSpeeds = chassisMovement.chassisSpeeds.ToRobotRelative(rot2d);
            }
            if (chassisMovement.driveOption == ChassisOptionEnums::DriveStateType::DRIVE_TO_PROCESSOR)
            {
                chassisMovement.yawAngle = m_endPose.Rotation().Degrees();
            }
        }
        else
        {
            // for if we initially do not have a target.
            chassisMovement.chassisSpeeds = chassisMovement.chassisSpeeds.ToRobotRelative(frc::Rotation2d(m_chassis->GetYaw()));
        }
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToFieldElement", "Error", m_endPose.Translation().Distance(m_currentPose.Translation()).value());
    }
    RobotState::GetInstance()->PublishStateChange(RobotStateChanges::DriveToFieldElementIsDone_Bool, IsDone());
    return m_robotDrive->UpdateSwerveModuleStates(chassisMovement);
}

void DriveToFieldElement::InitChassisMovement(ChassisMovement &chassisMovement)
{
    // initialize the same as holonomic drive
    chassisMovement.rawOmega = 0.0;
    chassisMovement.chassisSpeeds.vx = units::velocity::meters_per_second_t(0.0);
    chassisMovement.chassisSpeeds.vy = units::velocity::meters_per_second_t(0.0);
    chassisMovement.driveOption = GetDriveStateType();
    chassisMovement.controllerType = ChassisOptionEnums::AutonControllerType::HOLONOMIC;
    chassisMovement.headingOption = GetHeadingOption();
    chassisMovement.centerOfRotationOffset = frc::Translation2d();
    chassisMovement.noMovementOption = ChassisOptionEnums::NoMovementOption::STOP;
    chassisMovement.pathnamegains = ChassisOptionEnums::PathGainsType::LONG;
    chassisMovement.chassisSpeeds.omega = units::angular_velocity::radians_per_second_t(0);
    chassisMovement.checkTipping = false;
    chassisMovement.tippingTolerance = units::angle::degree_t(5.0);
    chassisMovement.tippingCorrection = -0.1;
    chassisMovement.targetPose = frc::Pose2d();
}

bool DriveToFieldElement::IsDone()
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

void DriveToFieldElement::CalculateFeedForward(ChassisMovement &chassisMovement)
{
    if (m_chassis != nullptr)
    {
        m_distanceError = m_currentPose.Translation().Distance(m_endPose.Translation());

        // Calculate feedforward speed based on distance
        units::velocity::meters_per_second_t feedforwardSpeed = 0.0_mps;
        if (m_distanceError > m_ffMinRadius)
        {
            double feedForwardScale = std::clamp(((m_distanceError - m_ffMinRadius) / (m_feedForwardRange)).value(), 0.0, 1.0);
            feedforwardSpeed = kMaxVelocity * feedForwardScale;
        }

        // Apply feedforward to the desired velocity
        frc::Translation2d translationError = m_endPose.Translation() - m_currentPose.Translation();
        frc::Rotation2d angleToTarget = translationError.Angle();

        chassisMovement.chassisSpeeds.vx = feedforwardSpeed * angleToTarget.Cos();
        chassisMovement.chassisSpeeds.vy = feedforwardSpeed * angleToTarget.Sin();
    }
}

void DriveToFieldElement::LogMoveInfo(ChassisMovement &moveInfo)
{
    LogChassisMovement::Print(moveInfo);
}
