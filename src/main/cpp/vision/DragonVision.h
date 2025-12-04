//====================================================================================================================================================
// Copyright 2022 Lake Orion Robotics FIRST Team 302
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
#include <map>
#include <memory>
#include <optional>
#include <vector>

#include "frc/geometry/Pose2d.h"

// FRC Includes
#include "frc/apriltag/AprilTagFieldLayout.h"

// Team 302 Includes
#include "vision/DragonLimelight.h"
#include "vision/DragonVisionPoseEstimatorStruct.h"
#include "vision/VisionPose.h"

#include "fielddata/FieldConstants.h"

#include <frc2/command/SubsystemBase.h>

// Developer documentation:
// @file DragonVision.h
// @brief High-level vision subsystem interface.
//
// Purpose:
//   DragonVision centralizes camera and vision processing integration for the robot.
//   It manages multiple camera sources (Limelight, Quest), provides health checks,
//   selects/sets pipelines, and exposes fused or per-camera pose estimates to the
//   rest of the robot code.
//
// Responsibilities:
//   - Maintain a map of camera instances and a single Quest instance.
//   - Provide methods to query vision targets (AprilTags, object detection) and
//     to obtain fused or camera-specific robot poses.
//   - Offer health checks and pipeline control for cameras.
//   - Periodically update/refresh NetworkTables and any internal state.
//
// Usage notes:
//   - AddLimelight/AddQuest should be called during robot initialization to register
//     cameras with this subsystem.
//   - GetRobotPositionMegaTag* returns an optional VisionPose; callers should check
//     for validity before use.
//   - Keep enum and NT key mappings synchronized with camera wrappers and network
//     table consumers to avoid mismatches.
//
// Class summary:
//   DragonVision : frc2::SubsystemBase
//     - Public API: AddLimelight, AddQuest, GetRobotPositionMegaTag*, HealthCheck, SetPipeline
//     - Threading: callers should assume single-threaded access from the robot loop.
//
// Notes:
//   - Prefer using VisionTargetOption and DRAGON_LIMELIGHT_* enums for camera/pipeline
//     selection to improve readability and reduce magic constants.
//   - Avoid reordering enum values if they are persisted or communicated via NT.

class DragonQuest;

class DragonVision : public frc2::SubsystemBase
{
public:
    /// @brief Get the singleton instance of DragonVision.
    /// @note Not thread-safe for initialization; if called concurrently during startup,
    ///       callers must ensure proper synchronization. The instance is leaked intentionally
    ///       for program lifetime management.
    /// @return Pointer to the DragonVision singleton instance.
    static DragonVision *GetDragonVision();

    /// @brief Get the AprilTag field layout used by vision code.
    /// @return Cached frc::AprilTagFieldLayout for the configured field (loads on first use).
    static frc::AprilTagFieldLayout GetAprilTagLayout();

    enum VISION_ELEMENT
    {
        ALGAE,
        BARGE,
        CORAL,
        CORAL_STATION,
        PROCESSOR,
        REEF,
        NEAREST_APRILTAG
    };

    /// @brief Register a Limelight camera with the vision manager.
    /// @param camera Raw pointer to DragonLimelight to add (ownership NOT transferred).
    /// @param usage Category/usage enum for this camera (affects pipeline selection and queries).
    /// @note The manager stores the raw pointer; lifetime must be managed by caller.
    void AddLimelight(DragonLimelight *camera, DRAGON_LIMELIGHT_CAMERA_USAGE usage);

    /// @brief Register the DragonQuest vision subsystem.
    /// @param quest Raw pointer to the DragonQuest instance (ownership NOT transferred).
    void AddQuest(DragonQuest *quest);

    static frc::AprilTagFieldLayout m_aprilTagLayout;

    /// @brief Aggregate AprilTag detections across cameras and select according to option.
    /// @param option Selection option (e.g., CLOSEST_VALID_TARGET).
    /// @param validTag List of FieldAprilTagIDs considered valid for selection.
    /// @return Vector of unique_ptr<DragonVisionStruct> containing selected targets.
    /// @note Ownership of returned pointers is transferred to the caller.
    std::vector<std::unique_ptr<DragonVisionStruct>> GetAprilTagVisionTargetInfo(VisionTargetOption option,
                                                                                 const std::vector<FieldAprilTagIDs> &validTag) const;

    /// @brief Aggregate object-detection results across cameras and select according to option.
    /// @param option Selection option (e.g., CLOSEST_VALID_TARGET).
    /// @param validClasses Vector of class IDs considered valid for selection.
    /// @return Vector of unique_ptr<DragonVisionStruct> containing selected targets.
    /// @note Ownership of returned pointers is transferred to the caller.
    std::vector<std::unique_ptr<DragonVisionStruct>> GetObjectDetectionTargetInfo(VisionTargetOption option,
                                                                                  const std::vector<int> &validClasses) const;

    /// @brief Health check for cameras by usage category.
    /// @param position Usage category to check (e.g., APRIL_TAGS).
    /// @return true if all matching, running cameras report healthy; false otherwise.
    bool HealthCheck(DRAGON_LIMELIGHT_CAMERA_USAGE position);

    /// @brief Health check for a single camera by identifier.
    /// @param identifier Camera identifier to check.
    /// @return true if the camera exists and reports running; false otherwise.
    bool HealthCheck(DRAGON_LIMELIGHT_CAMERA_IDENTIFIER identifier);

    /// @brief Set the processing pipeline for matching cameras.
    /// @param position Usage/category for cameras to update.
    /// @param pipeline Pipeline enum value to set.
    /// @note Only running cameras will be updated.
    void SetPipeline(DRAGON_LIMELIGHT_CAMERA_USAGE position, DRAGON_LIMELIGHT_PIPELINE pipeline);

    /// @brief Get robot pose estimate derived from MegaTag1 detections across cameras.
    /// @return Optional VisionPose; std::nullopt if no reliable estimation available.
    std::optional<VisionPose> GetRobotPositionMegaTag1();

    /// @brief Get robot pose estimate derived from MegaTag2 detections across cameras.
    /// @return Optional VisionPose; std::nullopt if no reliable estimation available.
    std::optional<VisionPose> GetRobotPositionMegaTag2();

    /// @brief Get robot pose estimate derived from Quest detections.
    /// @return DragonVisionPoseEstimatorStruct - confidence level indicates the usefulness of the pose.
    DragonVisionPoseEstimatorStruct GetRobotPositionQuest();

private:
    /// @brief Constructor (private for singleton).
    DragonVision();
    ~DragonVision() = default;

    /// @brief Distribute a Pose2d to vision subsystems that accept external robot pose.
    /// @param pose The pose to set (frc::Pose2d).
    /// @note Updates running limelights and the registered DragonQuest instance.
    void SetRobotPose(const frc::Pose2d &pose);

    /// @brief Return running limelights that match the provided usage/category.
    /// @param usage Usage/category to filter.
    /// @return Vector of pointers to running DragonLimelight instances.
    std::vector<DragonLimelight *> GetLimelights(DRAGON_LIMELIGHT_CAMERA_USAGE usage) const;

    /// @brief Lookup a limelight by its identifier.
    /// @param identifier Camera identifier to search for.
    /// @return Pointer to the DragonLimelight if found and running; nullptr otherwise.
    DragonLimelight *GetLimelightFromIdentifier(DRAGON_LIMELIGHT_CAMERA_IDENTIFIER identifier) const;

    /// @brief Return the registered DragonQuest instance (may be nullptr).
    /// @return Raw pointer to DragonQuest (no ownership transfer).
    DragonQuest *GetQuest() const { return m_dragonQuest; };

    /// @brief Singleton instance pointer.
    static DragonVision *m_dragonVision;

    /// @brief Map of usage/category to camera instances.
    /// @note Raw pointers stored; lifetime managed externally.
    std::multimap<DRAGON_LIMELIGHT_CAMERA_USAGE, DragonLimelight *> m_dragonLimelightMap;

    /// @brief Registered DragonQuest instance (raw pointer, may be null).
    DragonQuest *m_dragonQuest = nullptr;

    /// @brief Tracks whether an initial pose has been set on vision subsystems.
    bool m_initialPoseSet = false;
};
