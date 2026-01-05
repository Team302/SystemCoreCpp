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

// C++ Includes
#include <string>
#include <vector>

// FRC includes
#include "frc/DriverStation.h"
#include "frc/RobotBase.h"
#include "frc/Timer.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation3d.h"
#include "networktables/DoubleArrayTopic.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "units/angle.h"
#include "units/length.h"
#include "units/time.h"

// Team 302 includes
#include "chassis/ChassisConfigMgr.h"
#include "utils/logging/debug/Logger.h"
#include "vision/DragonLimelight.h"
#include "vision/DragonVision.h"

// Third Party Includes
#include "Limelight/LimelightHelpers.h"

namespace
{
    bool IsValidAprilTag(const std::vector<FieldAprilTagIDs> &validTags, int tagID);
    bool IsValidObjectClass(const std::vector<int> &validClasses, int classID);
    std::optional<std::pair<double, double>> GetStandardDeviationsForMetaTag1PoseEstimation(int ntags, double targetAreaPercent);
}

///-----------------------------------------------------------------------------------
/// Method:         DragonLimelight (constructor)
/// Description:    Create the object
///-----------------------------------------------------------------------------------
/// ----------------------------------------------------------------------------------
/// @brief Construct a DragonLimelight object.
/// @details Initializes network table handle, camera pose, chassis pointer, sets LED/camera/pipeline modes,
///          starts a health timer and port forwarding for Limelight access.
/// @param networkTableName NetworkTable name for the Limelight instance (sanitized internally)
/// @param identifier enum identifying which physical limelight this represents
/// @param cameraType limelight camera type (unused in current implementation)
/// @param cameraUsage whether this camera is used for odometry, vision, etc. (unused here)
/// @param mountingXOffset forward offset in inches from robot center
/// @param mountingYOffset left offset in inches from robot center
/// @param mountingZOffset up offset in inches from robot center
/// @param pitch camera pitch in degrees
/// @param yaw camera yaw in degrees
/// @param roll camera roll in degrees
/// @param initialPipeline starting pipeline to select on the Limelight
/// @param ledMode initial LED mode to set
/// @param camMode initial camera mode to set
DragonLimelight::DragonLimelight(
    std::string networkTableName, /// <I> networkTableName
    DRAGON_LIMELIGHT_CAMERA_IDENTIFIER identifier,
    DRAGON_LIMELIGHT_CAMERA_TYPE cameraType,
    DRAGON_LIMELIGHT_CAMERA_USAGE cameraUsage,
    units::length::inch_t mountingXOffset,     /// <I> x offset of cam from robot center (forward relative to robot)
    units::length::inch_t mountingYOffset,     /// <I> y offset of cam from robot center (left relative to robot)
    units::length::inch_t mountingZOffset,     /// <I> z offset of cam from robot center (up relative to robot)
    units::angle::degree_t pitch,              /// <I> - Pitch of camera
    units::angle::degree_t yaw,                /// <I> - Yaw of camera
    units::angle::degree_t roll,               /// <I> - Roll of camera
    DRAGON_LIMELIGHT_PIPELINE initialPipeline, /// <I> enum for pipeline
    DRAGON_LIMELIGHT_LED_MODE ledMode,
    DRAGON_LIMELIGHT_CAM_MODE camMode) : m_identifier(identifier),
                                         m_limelightNT(nt::NetworkTableInstance::GetDefault().GetTable(LimelightHelpers::sanitizeName(std::string(networkTableName)))),
                                         m_chassis(ChassisConfigMgr::GetInstance()->GetSwerveChassis()),
                                         m_cameraPose(frc::Pose3d(mountingXOffset, mountingYOffset, mountingZOffset, frc::Rotation3d(roll, pitch, yaw)))
{
    SetLEDMode(ledMode);
    SetCamMode(camMode);
    SetPipeline(initialPipeline);
    SetCameraPose_RobotSpace(mountingXOffset.to<double>(), mountingYOffset.to<double>(), mountingZOffset.to<double>(), roll.to<double>(), pitch.to<double>(), yaw.to<double>());
    m_cameraName = LimelightHelpers::sanitizeName(std::string(networkTableName));
    m_healthTimer = new frc::Timer();
    for (int port = 5800; port <= 5809; port++)
    {
        wpi::PortForwarder::GetInstance().Add(port + static_cast<int>(identifier), "limelight.local", port);
    }
}

/// ----------------------------------------------------------------------------------
/// @brief Check whether the Limelight is running/responding.
/// @details Uses heartbeat value exposed by Limelight network table and a short local timer.
///          In simulation always returns true.
/// @return true if Limelight is considered healthy and updating; false otherwise.
/// ----------------------------------------------------------------------------------
bool DragonLimelight::IsLimelightRunning()
{
    if (frc::RobotBase::IsSimulation())
    {
        return true; // In simulation, we don't have a limelight, so just return true
    }

    auto nt = m_limelightNT.get();
    if (nt != nullptr)
    {
        double currentHb = nt->GetNumber("hb", START_HB);

        // check if heartbeat has ever been set and network table is not default
        if (currentHb == START_HB)
        {
            return false;
        }
        else if (m_lastHeartbeat != currentHb)
        {
            m_lastHeartbeat = currentHb;
            m_healthTimer->Reset(); // reset when we see a new heartbeat
            m_healthTimer->Start();
            return true;
        }
        else if (m_healthTimer->Get().to<double>() < 0.5) // if we haven't seen a new heartbeat in 0.5 seconds
        {
            return true;
        }
    }
    return false;
}

/// ----------------------------------------------------------------------------------
/// @brief Retrieve detected AprilTag targets from Limelight.
/// @param validAprilTagIDs Optional list of field april tag IDs to filter results. If empty, all tags returned.
/// @return vector of DragonVisionStruct unique_ptr for each valid AprilTag detection. Empty if NT not available.
/// @notes Each DragonVisionStruct contains offsets, area, latency and distances computed by Limelight helpers.
/// ----------------------------------------------------------------------------------
std::vector<std::unique_ptr<DragonVisionStruct>> DragonLimelight::GetAprilTagVisionTargetInfo(const std::vector<FieldAprilTagIDs> &validAprilTagIDs) const
{
    std::vector<std::unique_ptr<DragonVisionStruct>> targets;
    auto nt = m_limelightNT.get();
    if (nt != nullptr)
    {
        auto aprilTags = LimelightHelpers::getRawFiducials(m_limelightNT);

        for (auto aprilTag : aprilTags)
        {
            auto isValid = IsValidAprilTag(validAprilTagIDs, aprilTag.id);

            if (!isValid)
            {
                continue; // skip this tag
            }

            auto aprilTagValue = std::make_unique<DragonVisionStruct>();
            aprilTagValue.get()->aprilTagData.tagID = static_cast<FieldAprilTagIDs>(aprilTag.id);
            aprilTagValue.get()->targetType = DragonTargetType::APRIL_TAG;
            aprilTagValue.get()->horizontalOffset = units::angle::degree_t(aprilTag.txnc);
            aprilTagValue.get()->verticalOffset = units::angle::degree_t(aprilTag.tync);
            aprilTagValue.get()->targetAreaPercent = aprilTag.ta;
            aprilTagValue.get()->pipelineLatency = units::millisecond_t(LimelightHelpers::getLatency_Pipeline(m_limelightNT) + LimelightHelpers::getLatency_Capture(m_limelightNT));
            aprilTagValue.get()->aprilTagData.distToCamera = units::length::meter_t(aprilTag.distToCamera);
            aprilTagValue.get()->aprilTagData.distToRobot = units::length::meter_t(aprilTag.distToRobot);
            aprilTagValue.get()->aprilTagData.ambiguity = aprilTag.ambiguity;

            targets.emplace_back(std::move(aprilTagValue));
        }
    }

    return targets;
}

/// ----------------------------------------------------------------------------------
/// @brief Retrieve object detection results from Limelight (e.g., neural detector).
/// @param validClasses Optional filter of class ids to keep; if empty, all detections returned.
/// @return vector of DragonVisionStruct unique_ptr for each valid object detection. Empty if NT not available.
/// @notes Corners and class id are populated in the returned structs.
/// ----------------------------------------------------------------------------------
std::vector<std::unique_ptr<DragonVisionStruct>> DragonLimelight::GetObjectDetectionTargetInfo(const std::vector<int> &validClasses) const
{
    std::vector<std::unique_ptr<DragonVisionStruct>> targets;
    auto nt = m_limelightNT.get();
    if (nt != nullptr)
    {
        auto objects = LimelightHelpers::getRawDetections(m_limelightNT);

        for (auto object : objects)
        {
            auto isValid = IsValidObjectClass(validClasses, object.classId);

            if (!isValid)
            {
                continue; // skip this tag
            }

            auto objectValue = std::make_unique<DragonVisionStruct>();
            objectValue.get()->objectDetectionData.classID = object.classId;
            objectValue.get()->targetType = DragonTargetType::OBJECT_DETECTION;
            objectValue.get()->horizontalOffset = units::angle::degree_t(object.txnc);
            objectValue.get()->verticalOffset = units::angle::degree_t(object.tync);
            objectValue.get()->targetAreaPercent = object.ta;
            objectValue.get()->pipelineLatency = units::millisecond_t(LimelightHelpers::getLatency_Pipeline(m_limelightNT) + LimelightHelpers::getLatency_Capture(m_limelightNT));
            objectValue.get()->objectDetectionData.corner0X = object.corner0_X;
            objectValue.get()->objectDetectionData.corner0Y = object.corner0_Y;
            objectValue.get()->objectDetectionData.corner1X = object.corner1_X;
            objectValue.get()->objectDetectionData.corner1Y = object.corner1_Y;
            objectValue.get()->objectDetectionData.corner2X = object.corner2_X;
            objectValue.get()->objectDetectionData.corner2Y = object.corner2_Y;
            objectValue.get()->objectDetectionData.corner3X = object.corner3_X;
            objectValue.get()->objectDetectionData.corner3Y = object.corner3_Y;

            targets.emplace_back(std::move(objectValue));
        }
    }

    return targets;
}

/**
 * @brief Get the Pose object for the current location of the robot
 * https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization
 */
/// ----------------------------------------------------------------------------------
/// @brief High-level entry to request a pose estimate from Limelight for odometry.
/// @param useMegatag2 if true, request MegaTag2 pose estimation path; otherwise use MegaTag1 path.
/// @return optional VisionPose when Limelight has a valid pose estimate; std::nullopt otherwise.
/// @notes Adjusts Limelight IMU mode for best results depending on which MegaTag method is used.
/// ----------------------------------------------------------------------------------
std::optional<VisionPose> DragonLimelight::EstimatePoseOdometryLimelight(bool useMegatag2)
{
    if (frc::RobotBase::IsSimulation())
    {
        return std::nullopt;
    }

    auto mode = static_cast<int>(LIMELIGHT_IMU_MODE::USE_EXTERNAL_IMU_ONLY); // Chief Delphi answer says perfect portrait pose doesn't work with internal IMU
    LimelightHelpers::SetIMUMode(m_cameraName, mode);

    if (useMegatag2)
    {
        return GetMegaTag2Pose();
    }
    else
    {
        return GetMegaTag1Pose();
    }
    return std::nullopt;
}

/// ----------------------------------------------------------------------------------
/// @brief Get pose estimate using the MegaTag1/standard Limelight pose estimate API.
/// @return optional VisionPose populated from Limelight pose if tagCount > 0 and valid deviations are computable.
/// @sideeffects If robot pose has not been set, SetRobotPose will be invoked using the estimate's 2D pose.
/// ----------------------------------------------------------------------------------
std::optional<VisionPose> DragonLimelight::GetMegaTag1Pose()
{
    auto limelightMeasurement = LimelightHelpers::getBotPoseEstimate_wpiBlue(m_cameraName);

    if (limelightMeasurement.tagCount == 0)
    {
        return std::nullopt;
    }
    auto deviations = GetStandardDeviationsForMetaTag1PoseEstimation(limelightMeasurement.tagCount, limelightMeasurement.avgTagArea);
    if (!deviations.has_value())
    {
        return std::nullopt;
    }

    auto pose3d = frc::Pose3d{limelightMeasurement.pose};

    units::time::millisecond_t currentTime = frc::Timer::GetFPGATimestamp();
    units::time::millisecond_t timestamp = currentTime - units::millisecond_t(limelightMeasurement.timestampSeconds / 1000.0);

    double xyStds = deviations.value().first;
    double degStds = deviations.value().second;

    m_megatag1PosBool = true;
    m_megatag1Pos = {pose3d, timestamp, {xyStds, xyStds, degStds}, PoseEstimationStrategy::MEGA_TAG};
    if (!m_robotPoseSet)
    {
        SetRobotPose(pose3d.ToPose2d());
    }
    return m_megatag1Pos;
}

/// ----------------------------------------------------------------------------------
/// @brief Get pose estimate using the MegaTag2 specialized API.
/// @details Requires an initial robot pose to be set (will attempt to get MegaTag1 pose first if needed).
/// @return optional VisionPose populated from MegaTag2 pose estimate API; std::nullopt on failure.
/// ----------------------------------------------------------------------------------
std::optional<VisionPose> DragonLimelight::GetMegaTag2Pose()
{
    if (!m_robotPoseSet)
    {
        auto megatag1pose = GetMegaTag1Pose();
        if (!megatag1pose.has_value())
        {
            return std::nullopt;
        }
    }
    // Get the pose estimate
    auto mode = frc::DriverStation::IsDisabled() ? static_cast<int>(LIMELIGHT_IMU_MODE::USE_EXTERNAL_IMU_AND_FUSE_WITH_INTERNAL_IMU) : static_cast<int>(LIMELIGHT_IMU_MODE::USE_INTERNAL_IMU);
    LimelightHelpers::SetIMUMode(m_cameraName, mode);
    auto poseEstimate = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(m_cameraName);

    double xyStds = .7;
    double degStds = 9999999;
    m_megatag2PosBool = true;
    m_megatag2Pos = {frc::Pose3d{poseEstimate.pose},
                     poseEstimate.timestampSeconds,
                     {xyStds, xyStds, degStds},
                     PoseEstimationStrategy::MEGA_TAG_2};

    return m_megatag2Pos;
}

/// ----------------------------------------------------------------------------------
/// @brief Set Limelight LED mode.
/// @param mode enumeration controlling LED behavior (pipeline, blink, on, off)
/// @notes Uses LimelightHelpers wrappers to send commands to the camera.
/// ----------------------------------------------------------------------------------
void DragonLimelight::SetLEDMode(DRAGON_LIMELIGHT_LED_MODE mode)
{
    switch (mode)
    {
    case DRAGON_LIMELIGHT_LED_MODE::LED_PIPELINE_CONTROL:
        LimelightHelpers::setLEDMode_PipelineControl(m_cameraName);
        break;
    case DRAGON_LIMELIGHT_LED_MODE::LED_BLINK:
        LimelightHelpers::setLEDMode_ForceBlink(m_cameraName);
        break;
    case DRAGON_LIMELIGHT_LED_MODE::LED_ON:
        LimelightHelpers::setLEDMode_ForceOn(m_cameraName);
        break;
    case DRAGON_LIMELIGHT_LED_MODE::LED_OFF: // default to off
    default:
        LimelightHelpers::setLEDMode_ForceOff(m_cameraName);
        break;
    }
}

/// ----------------------------------------------------------------------------------
/// @brief Set Limelight camera mode (e.g., vision vs driver camera).
/// @param mode enum value to put into the camera's camMode NT entry.
/// ----------------------------------------------------------------------------------
void DragonLimelight::SetCamMode(DRAGON_LIMELIGHT_CAM_MODE mode)
{
    auto nt = m_limelightNT.get();
    if (nt != nullptr)
    {
        nt->PutNumber("camMode", static_cast<int>(mode));
    }
}

/**
 * @brief Update the pipeline index, this assumes that all of your limelights have the same pipeline at each index
 */
/// ----------------------------------------------------------------------------------
/// @brief Set the Limelight pipeline index.
/// @param pipeline enum index for the selected pipeline
/// ----------------------------------------------------------------------------------
void DragonLimelight::SetPipeline(DRAGON_LIMELIGHT_PIPELINE pipeline)
{
    m_pipeline = pipeline;
    LimelightHelpers::setPipelineIndex(m_cameraName, static_cast<int>(pipeline));
}

/// ----------------------------------------------------------------------------------
/// @brief Set the Limelight priority tag id used for pose selection (Limelight helper wrapper).
/// @param id Field AprilTag id to give priority to.
/// ----------------------------------------------------------------------------------
void DragonLimelight::SetPriorityTagID(int id)
{
    LimelightHelpers::setPriorityTagID(m_cameraName, id);
}

/// ----------------------------------------------------------------------------------
/// @brief Publish the camera transform relative to the robot to the Limelight.
/// @param forward forward offset (inches)
/// @param left left offset (inches)
/// @param up up offset (inches)
/// @param roll roll (degrees)
/// @param pitch pitch (degrees)
/// @param yaw yaw (degrees)
/// @notes Calls LimelightHelpers::setCameraPose_RobotSpace.
/// ----------------------------------------------------------------------------------
void DragonLimelight::SetCameraPose_RobotSpace(double forward, double left, double up, double roll, double pitch, double yaw)
{
    LimelightHelpers::setCameraPose_RobotSpace(m_cameraName, forward, left, up, roll, pitch, yaw);
}

namespace
{
    /// ----------------------------------------------------------------------------------
    /// @brief Determine if an AprilTag id is in the allowed list.
    /// @param validTags allowed FieldAprilTagIDs list; empty means accept all.
    /// @param tagID numeric id to test
    /// @return true if tagID is accepted, false otherwise.
    /// ----------------------------------------------------------------------------------
    bool IsValidAprilTag(const std::vector<FieldAprilTagIDs> &validTags, int tagID)
    {
        if (validTags.empty())
        {
            return true;
        }

        auto it = std::find_if(validTags.begin(), validTags.end(),
                               [&tagID](const FieldAprilTagIDs &validTags)
                               { return static_cast<int>(validTags) == tagID; });

        return it != validTags.end();
    }

    /// ----------------------------------------------------------------------------------
    /// @brief Determine if an object detection class id is in the allowed list.
    /// @param validClasses allowed class ids; empty means accept all.
    /// @param classID numeric class id to test
    /// @return true if classID is accepted, false otherwise.
    /// ----------------------------------------------------------------------------------
    bool IsValidObjectClass(const std::vector<int> &validClasses, int classID)
    {
        if (validClasses.empty())
        {
            return true;
        }

        auto it = std::find_if(validClasses.begin(), validClasses.end(),
                               [&classID](const int &validClass)
                               { return validClass == classID; });

        return it != validClasses.end();
    }

    /// ----------------------------------------------------------------------------------
    /// @brief Compute conservative standard deviations for pose (x/y in meters, yaw in degrees)
    ///        based on number of tags seen and their average area reported by Limelight.
    /// @param ntags number of tags used in the pose estimate
    /// @param targetAreaPercent average tag area (normalized float from Limelight)
    /// @return pair(xyStdMeters, yawStdDegrees) if computable, std::nullopt if pose not reliable.
    /// ----------------------------------------------------------------------------------
    std::optional<std::pair<double, double>> GetStandardDeviationsForMetaTag1PoseEstimation(int ntags, double targetAreaPercent)
    {

        if (ntags == 0)
        {
            return std::nullopt;
        }

        double xyStds = 0.5; // assume we see 2 or more tags
        double degStds = 6;  // assume we see 2 or more tags
        LimelightHelpers::PoseEstimate limelightMeasurement = LimelightHelpers::getBotPoseEstimate_wpiBlue("");

        if (limelightMeasurement.tagCount == 1)
        {
            if (limelightMeasurement.avgTagArea > 0.8)
            {
                xyStds = 1.0;
                degStds = 12;
            }
            else if (limelightMeasurement.avgTagArea > 0.1)
            {
                xyStds = 2.0;
                degStds = 30;
            }
            else
            {
                return std::nullopt;
            }
        }
        return std::make_pair(xyStds, degStds);
    }
}

/// ----------------------------------------------------------------------------------
/// @brief Configure the robot orientation that the Limelight should use for pose fusion.
/// @details Sends yaw, yaw rate and camera pitch/roll to the Limelight so it can fuse orientation.
/// @param pose 2D robot pose used to set initial yaw/orientation for the Limelight.
/// ----------------------------------------------------------------------------------
void DragonLimelight::SetRobotPose(const frc::Pose2d &pose)
{
    auto yawrate = 0.0;
    auto pitch = 0.0;
    auto pitchrate = 0.0;
    auto roll = 0.0;
    auto rollrate = 0.0;
    if (m_chassis != nullptr)
    {
        yawrate = ChassisConfigMgr::GetInstance()->GetRotationRateDegreesPerSecond();
        pitch = m_cameraPose.Rotation().Y().value();
        roll = m_cameraPose.Rotation().X().value();
    }

    LimelightHelpers::SetRobotOrientation(m_cameraName,
                                          pose.Rotation().Degrees().value(),
                                          yawrate,
                                          pitch,
                                          pitchrate,
                                          roll,
                                          rollrate);
    m_robotPoseSet = true;
}