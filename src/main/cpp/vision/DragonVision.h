
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

#include "frc/geometry/Pose3d.h"
#include "units/angular_velocity.h"

// FRC Includes
#include "frc/apriltag/AprilTagFieldLayout.h"
#include "frc/apriltag/AprilTagFields.h"

// Team 302 Includes
#include "vision/DragonVisionStructs.h"
#include "vision/DragonLimelight.h"

#include "configs/RobotElementNames.h"
#include "fielddata/FieldConstants.h"

#include "units/angular_velocity.h"
class DragonVision
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

    std::optional<frc::Pose3d> GetAprilTagPose(FieldConstants::AprilTagIDs tagId) const;

    /// @brief gets the field position of the robot (right blue driverstation origin)
    /// @return std::optional<VisionPose> - the estimated position, timestamp of estimation, and confidence as array of std devs
    std::optional<VisionPose> GetRobotPosition();

    /// @brief gets the field position of the robot (Limelight only) (right blue driverstation origin)
    /// @return std::optional<VisionPose> - the estimated position, timestamp of estimation, and confidence as array of std devs
    std::optional<VisionPose> GetRobotPositionMegaTag2(units::angle::degree_t yaw, units::angular_velocity::degrees_per_second_t yawRate, units::angle::degree_t pitch, units::angular_velocity::degrees_per_second_t pitchRate, units::angle::degree_t roll, units::angular_velocity::degrees_per_second_t rollRate);

    /// @brief gets the distances and angles to the specified field element based on AprilTag readings or detections
    /// @param element the specified game element to get data to
    /// @return std::optional<VisionData> - a transform containg x, y, z distances and yaw, pitch, roll to target, and AprilTag Id
    std::optional<VisionData> GetVisionData(VISION_ELEMENT element);

    /// @brief adds a camera at the specified position to DragonVision
    /// @param camera pointer to the camera object that should be added
    /// @param position the physical position of the camera
    void AddLimelight(DragonLimelight *camera, DRAGON_LIMELIGHT_CAMERA_USAGE usage);

    /// @brief calculates the pose from other methods of vision
    /// @return std::optional<frc::Pose2d>
    std::optional<frc::Pose2d> CalcVisionPose();

    std::vector<DragonVisionPoseEstimator *> GetPoseEstimators() { return m_poseEstimators; };

    // raw data methods

    std::optional<units::angle::degree_t> GetTargetYaw(DRAGON_LIMELIGHT_CAMERA_USAGE position);
    std::optional<units::angle::degree_t> GetTargetPitch(DRAGON_LIMELIGHT_CAMERA_USAGE position);
    std::optional<units::angle::degree_t> GetTargetSkew(DRAGON_LIMELIGHT_CAMERA_USAGE position);

    std::optional<int> GetAprilTagID(DRAGON_LIMELIGHT_CAMERA_USAGE position);
    bool HasTarget(DRAGON_LIMELIGHT_CAMERA_USAGE position);
    std::optional<double> GetTargetArea(DRAGON_LIMELIGHT_CAMERA_USAGE position);

    units::angle::degree_t GetTx(DRAGON_LIMELIGHT_CAMERA_USAGE position);
    units::angle::degree_t GetTy(DRAGON_LIMELIGHT_CAMERA_USAGE position);

    static frc::AprilTagFieldLayout m_aprilTagLayout;

    void testAndLogVisionData();

    bool HealthCheck(DRAGON_LIMELIGHT_CAMERA_USAGE position);
    bool HealthCheck(DRAGON_LIMELIGHT_CAMERA_IDENTIFIER identifier);

    void SetPipeline(DRAGON_LIMELIGHT_CAMERA_USAGE position, DRAGON_LIMELIGHT_PIPELINE pipeline);

private:
    DragonVision();
    ~DragonVision() = default;

    std::vector<DragonLimelight *> GetCameras(DRAGON_LIMELIGHT_CAMERA_USAGE usage) const;
    DragonLimelight *GetCameras(DRAGON_LIMELIGHT_CAMERA_IDENTIFIER identifier) const;

    std::optional<VisionData> GetVisionDataFromAlgae(VISION_ELEMENT element);
    std::optional<VisionData> GetVisionDataFromElement(VISION_ELEMENT element);
    std::optional<VisionData> GetVisionDataToNearestTag();
    std::vector<int> GetReefTags(frc::DriverStation::Alliance allianceColor) const;
    std::vector<int> GetCoralStationsTags(frc::DriverStation::Alliance allianceColor) const;
    std::vector<int> GetProcessorTags(frc::DriverStation::Alliance allianceColor) const;
    std::vector<int> GetBargeTags(frc::DriverStation::Alliance allianceColor) const;

    std::optional<VisionData> GetVisionDataToNearestFieldElementAprilTag(VISION_ELEMENT element);
    std::optional<VisionData> SingleTagToElement(frc::Pose3d elementPose, int idToSearch);
    std::optional<VisionData> GetRawVisionDataFromObject(std::vector<DragonLimelight *> cameras, DRAGON_LIMELIGHT_PIPELINE pipeline);

    static DragonVision *m_dragonVision;
    std::multimap<DRAGON_LIMELIGHT_CAMERA_USAGE, DragonLimelight *> m_dragonLimelightMap;
    std::vector<DragonVisionPoseEstimator *> m_poseEstimators;
};
