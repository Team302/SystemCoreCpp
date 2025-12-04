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

/**
 * @file DragonVisionPoseEstimator.cpp
 * @brief Implements DragonVisionPoseEstimator: bridges the vision subsystem to the robot
 *        chassis pose estimator.
 *
 * Responsibilities:
 *  - Acquire and maintain a reference to the DragonVision subsystem.
 *  - Initialize the robot pose using reliable vision tag observations (prefers MegaTag1 then MegaTag2).
 *  - Periodically feed vision-derived pose measurements into the chassis's estimator.
 *  - Maintain DragonQuest heartbeat/NT refresh while active.
 *
 * Current behavior:
 *  - CalculateInitialPose(): attempts to set an initial pose only when MegaTag1 and MegaTag2
 *    provide consistent estimates (MegaTag1 is preferred first; MegaTag2 confirms).
 *  - AddVisionMeasurements(): currently uses MegaTag2 for ongoing vision updates and calls
 *    m_chassis->AddVisionMeasurement(...) with the measured pose, timestamp, and std deviations.
 *
 * Dependencies:
 *  - DragonVision: supplies tag-based robot pose estimates and timestamps.
 *  - DragonQuest: heartbeat/NT refresh calls are performed in Execute().
 *  - ChassisConfigMgr / SwerveChassis: receives vision measurements via AddVisionMeasurement.
 *
 * Known limitations & assumptions:
 *  - Not thread safe; intended to run on the main robot thread / scheduler.
 *  - Timestamp units: AddVisionMeasurements converts the DragonVision timestamp directly into
 *    units::second_t. Ensure DragonVision timestamps are in seconds (or adjust conversion accordingly).
 *  - Current continuous updates use MegaTag2; if additional estimators are added, AddVisionMeasurements
 *    should be extended to aggregate multiple vision sources and check confidence.
 *
 * Notes:
 *  - This file contains only implementation details — consult the header for public API.
 *  - Keep documentation near the implementation when behavior/assumptions change.
 */

/// Sorted includes: standard library first, then project headers (alphabetical).
#include <optional>

#include "chassis/ChassisConfigMgr.h"
#include "vision/DragonQuest.h"
#include "vision/DragonVision.h"
#include "vision/DragonVisionPoseEstimator.h"
#include "vision/DragonVisionPoseEstimatorStruct.h"

DragonVisionPoseEstimator::DragonVisionPoseEstimator()
{
    m_chassis = ChassisConfigMgr::GetInstance()->GetSwerveChassis();
}

/**
 * @brief Initialize vision pointer and attempt to compute initial pose.
 *
 * Ensures a DragonVision pointer is available and then calls CalculateInitialPose()
 * to set the chassis pose from reliable MegaTag observations.
 */
void DragonVisionPoseEstimator::Initialize()
{
    // Make sure we have a vision subsystem pointer.
    // If we don't, try to get one
    // If we still don't have one exit
    if (m_vision == nullptr)
    {
        m_vision = DragonVision::GetDragonVision();
    }
    if (m_vision == nullptr)
    {
        return;
    }

    CalculateInitialPose();
}

/**
 * @brief Main periodic execution.
 *
 * Ensures vision pointer availability, attempts to set an initial pose if needed,
 * maintains DragonQuest heartbeat/NT refresh, and pushes vision measurements to the chassis.
 */
void DragonVisionPoseEstimator::Execute()
{
    // Make sure we have a vision subsystem pointer.
    // If we don't, try to get one
    // If we still don't have one exit
    if (m_vision == nullptr)
    {
        m_vision = DragonVision::GetDragonVision();
    }
    if (m_vision == nullptr)
    {
        return;
    }

    if (!m_initialPoseSet)
    {
        CalculateInitialPose();
    }

    if (m_initialPoseSet == false)
    {
        return;
    }

    if (m_quest != nullptr)
    {
        m_quest->HandleHeartBeat();
        m_quest->RefreshNT();
    }

    AddVisionMeasurements();
}

/**
 * @brief Indicate whether this command is finished.
 * @returns false (this is intended to run continuously while scheduled).
 */
bool DragonVisionPoseEstimator::IsFinished()
{
    return false;
}

/**
 * @brief Get the current chassis pose.
 * @returns the chassis pose if available, otherwise a default Pose2d.
 */
frc::Pose2d DragonVisionPoseEstimator::GetPose() const
{
    return (m_chassis != nullptr) ? m_chassis->GetPose() : frc::Pose2d{};
}

/**
 * @brief Reset the chassis position to the provided pose.
 *
 * Wrapper around chassis ResetPose to centralize pose reset logic used by vision.
 */
void DragonVisionPoseEstimator::ResetPosition(const frc::Pose2d &pose)
{
    if (m_chassis != nullptr)
    {
        m_chassis->ResetPose(pose);
    }
}

/**
 * @brief Attempt to compute and set the initial robot pose from vision tags.
 *
 * Strategy:
 *  - Prefer MegaTag1; only if MegaTag1 and MegaTag2 both provide estimates do we mark
 *    the initial pose as set. This reduces the chance of incorrect resets.
 */
void DragonVisionPoseEstimator::CalculateInitialPose()
{
    if (m_vision == nullptr)
    {
        return;
    }

    m_vision = DragonVision::GetDragonVision();
    // try making sure MegaTag1 has a good position before resetting pose to avoid screwing up MegaTag2 && Quest
    auto megaTag1Position = m_vision->GetRobotPositionMegaTag1();
    if (megaTag1Position.has_value())
    {
        ResetPosition(megaTag1Position.value().estimatedPose.ToPose2d());

        auto megaTag2Position = m_vision->GetRobotPositionMegaTag2();
        if (megaTag2Position.has_value())
        {
            ResetPosition(megaTag2Position.value().estimatedPose.ToPose2d());
            m_initialPoseSet = true;
        }
    }
}

/**
 * @brief Pull vision pose estimates and push them into the chassis estimator.
 *
 * Currently the method contains TODOs — implementation should iterate estimators
 * from the vision subsystem and call the chassis AddVisionMeasurement APIs.
 */
void DragonVisionPoseEstimator::AddVisionMeasurements()
{
    if (m_vision == nullptr)
    {
        return;
    }

    auto visPose = m_vision->GetRobotPositionMegaTag2();
    if (visPose.has_value())
    {
        m_chassis->AddVisionMeasurement(visPose.value().estimatedPose.ToPose2d(), units::second_t{visPose.value().timeStamp}, visPose.value().visionMeasurementStdDevs);
    }

    auto questPose = m_vision->GetRobotPositionQuest();
    if (questPose.m_confidenceLevel == DragonVisionPoseEstimatorStruct::ConfidenceLevel::HIGH)
    {
        m_chassis->AddVisionMeasurement(questPose.m_visionPose, questPose.m_timeStamp, questPose.m_stds);
    }
}
