
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
#include <string>

#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Pose3d.h"
#include "units/angular_velocity.h"

// FRC Includes
#include "frc/apriltag/AprilTagFieldLayout.h"
#include "frc/apriltag/AprilTagFields.h"

// Team 302 Includes
#include "vision/DragonLimelight.h"
#include "vision/VisionPose.h"

#include "configs/RobotElementNames.h"
#include "fielddata/FieldConstants.h"

#include "units/angular_velocity.h"

#include <frc2/command/SubsystemBase.h>

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

    static frc::AprilTagFieldLayout m_aprilTagLayout;

    std::vector<std::unique_ptr<DragonVisionStruct>> GetAprilTagVisionTargetInfo(VisionTargetOption option,
                                                                                 const std::vector<FieldAprilTagIDs> &validTag) const;
    std::vector<std::unique_ptr<DragonVisionStruct>> GetObjectDetectionTargetInfo(VisionTargetOption option,
                                                                                  const std::vector<int> &validClasses) const;

    bool HealthCheck(DRAGON_LIMELIGHT_CAMERA_USAGE position);
    bool HealthCheck(DRAGON_LIMELIGHT_CAMERA_IDENTIFIER identifier);

    void SetPipeline(DRAGON_LIMELIGHT_CAMERA_USAGE position, DRAGON_LIMELIGHT_PIPELINE pipeline);

    void Periodic() override;

    /// @brief gets the field position of the robot (right blue driverstation origin)
    /// @return std::optional<frc::Pose3d> - the estimated position, timestamp of estimation, and confidence as array of std devs
    std::optional<VisionPose> GetRobotPositionMegaTag1();

    /// @brief gets the field position of the robot (Limelight only) (right blue driverstation origin)
    /// @return std::optional<VisionPose> - the estimated position, timestamp of estimation, and confidence as array of std devs
    std::optional<VisionPose> GetRobotPositionMegaTag2();

private:
    DragonVision();
    ~DragonVision() = default;

    void SetRobotPose(const frc::Pose2d &pose);

    std::vector<DragonLimelight *> GetLimelights(DRAGON_LIMELIGHT_CAMERA_USAGE usage) const;
    DragonLimelight *GetLimelightFromIdentifier(DRAGON_LIMELIGHT_CAMERA_IDENTIFIER identifier) const;

    static DragonVision *m_dragonVision;
    std::multimap<DRAGON_LIMELIGHT_CAMERA_USAGE, DragonLimelight *> m_dragonLimelightMap;
    bool m_initialPoseSet = false;
};
