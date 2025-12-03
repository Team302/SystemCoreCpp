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
    static DragonVision *GetDragonVision();

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

    /// @brief adds a camera at the specified position to DragonVision
    /// @param camera pointer to the camera object that should be added
    /// @param position the physical position of the camera
    void AddLimelight(DragonLimelight *camera, DRAGON_LIMELIGHT_CAMERA_USAGE usage);
    void AddQuest(DragonQuest *quest);

    static frc::AprilTagFieldLayout m_aprilTagLayout;

    std::vector<std::unique_ptr<DragonVisionStruct>> GetAprilTagVisionTargetInfo(VisionTargetOption option,
                                                                                 const std::vector<FieldAprilTagIDs> &validTag) const;
    std::vector<std::unique_ptr<DragonVisionStruct>> GetObjectDetectionTargetInfo(VisionTargetOption option,
                                                                                  const std::vector<int> &validClasses) const;

    bool HealthCheck(DRAGON_LIMELIGHT_CAMERA_USAGE position);
    bool HealthCheck(DRAGON_LIMELIGHT_CAMERA_IDENTIFIER identifier);

    void SetPipeline(DRAGON_LIMELIGHT_CAMERA_USAGE position, DRAGON_LIMELIGHT_PIPELINE pipeline);

    /// @brief gets the field position of the robot (right blue driverstation origin)
    /// @return std::optional<frc::Pose3d> - the estimated position, timestamp of estimation, and confidence as array of std devs
    std::optional<VisionPose> GetRobotPositionMegaTag1();

    /// @brief gets the field position of the robot (Limelight only) (right blue driverstation origin)
    /// @return std::optional<frc::Pose3d> - the estimated position, timestamp of estimation, and confidence as array of std devs
    std::optional<VisionPose> GetRobotPositionMegaTag2();

private:
    DragonVision();
    ~DragonVision() = default;

    void SetRobotPose(const frc::Pose2d &pose);

    std::vector<DragonLimelight *> GetLimelights(DRAGON_LIMELIGHT_CAMERA_USAGE usage) const;
    DragonLimelight *GetLimelightFromIdentifier(DRAGON_LIMELIGHT_CAMERA_IDENTIFIER identifier) const;
    DragonQuest *GetQuest() const { return m_dragonQuest; };

    static DragonVision *m_dragonVision;
    std::multimap<DRAGON_LIMELIGHT_CAMERA_USAGE, DragonLimelight *> m_dragonLimelightMap;
    DragonQuest *m_dragonQuest = nullptr;

    bool m_initialPoseSet = false;
};
