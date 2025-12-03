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

// WPILib / third-party
#include "frc/geometry/Pose2d.h"
#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"

// Project headers
#include "chassis/ChassisConfigMgr.h"
#include "chassis/generated/CommandSwerveDrivetrain.h"
#include "vision/DragonVision.h"

class DragonQuest;

/**
 * @class DragonVisionPoseEstimator
 * @brief Fuses vision measurements with swerve drivetrain odometry to maintain an accurate field pose.
 *
 * This command periodically reads target observations from DragonVision and injects
 * corrected poses or measurements into the drivetrain pose estimator (via the swerve chassis)
 * to reduce odometry drift. It is intended to run while the robot is enabled and can be
 * scheduled continuously (does not normally terminate by itself).
 *
 * Key behaviors:
 * - Attempts to set an initial absolute pose when a reliable vision target is first seen
 *   (CalculateInitialPose / m_initialPoseSet).
 * - On each Execute(), valid vision measurements are transformed and handed to the chassis
 *   pose estimator using AddVisionMeasurements().
 * - ResetPosition() can be used to force a drivetrain pose reset from external code.
 *
 * Usage:
 * - Construct and schedule at robot initialization or when vision-based pose updates are desired.
 * - Ensure DragonVision provides properly timestamped/filtered detections for best results.
 */
class DragonVisionPoseEstimator : public frc2::CommandHelper<frc2::Command, DragonVisionPoseEstimator>
{
public:
    /**
     * @brief Construct the estimator command.
     *
     * Initializes pointers to the swerve chassis and DragonVision singletons.
     */
    DragonVisionPoseEstimator();
    ~DragonVisionPoseEstimator() = default;

    // FRC Command Lifecycle methods

    /**
     * @brief Called once when the command is scheduled.
     *
     * Prepare internal state and attempt to set initial pose if possible.
     */
    void Initialize() override;

    /**
     * @brief Called repeatedly while the command is scheduled.
     *
     * Reads camera detections and fuses them into the drivetrain pose estimator.
     * Should be non-blocking and handle intermittent/invalid measurements.
     */
    void Execute() override;

    /**
     * @brief Command termination condition.
     *
     * This command is intended to run continuously; override to true only for explicit termination.
     */
    bool IsFinished() override;

    /**
     * @brief Attempt to compute and apply an initial absolute pose from vision.
     *
     * Should be called once (or until successful) to align odometry with field coordinates.
     */
    void CalculateInitialPose();

    /**
     * @brief Force a reset of the drivetrain pose estimator.
     * @param pose The new pose (field coordinates) to set the drivetrain to.
     */
    void ResetPosition(const frc::Pose2d &pose);

    /**
     * @brief Return the current estimated pose from the drivetrain estimator.
     */
    frc::Pose2d GetPose() const;

private:
    /**
     * Pointer to the swerve drivetrain used to update/fetch odometry/pose estimator.
     */
    subsystems::CommandSwerveDrivetrain *m_chassis = ChassisConfigMgr::GetInstance()->GetSwerveChassis();

    /**
     * Pointer to the vision subsystem providing target detections.
     */
    DragonVision *m_vision = DragonVision::GetDragonVision();

    /**
     * Optional quest/goal tracker used by vision logic (may be nullptr until initialized).
     */
    DragonQuest *m_quest;

    /**
     * @brief Convert and add vision-based pose measurements to the drivetrain estimator.
     *
     * Responsible for validating detections, applying coordinate transforms, and
     * timestamping measurements before passing them to the chassis pose estimator.
     */
    void AddVisionMeasurements();

    /**
     * @brief True when an initial pose has been successfully set from vision.
     */
    bool m_initialPoseSet = false;
};