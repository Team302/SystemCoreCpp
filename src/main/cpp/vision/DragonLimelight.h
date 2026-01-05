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

// C++ Includes
#include <memory>
#include <string>
#include <vector>

// FRC includes
#include "fielddata/FieldConstants.h"
#include "frc/DriverStation.h"
#include "frc/Timer.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Pose3d.h"
#include "networktables/NetworkTable.h"
#include "units/angle.h"
#include "units/length.h"
#include "units/time.h"

// Team 302 includes
#include "chassis/ChassisConfigMgr.h"
#include "fielddata/FieldAprilTagIDs.h"
#include "vision/DragonVisionEnums.h"
#include "vision/DragonVisionStruct.h"
#include "vision/VisionPose.h"

// Third Party Includes

// DragonLimelight needs to be a child of DragonCamera
class DragonLimelight
{
public:
    ///-----------------------------------------------------------------------------------
    /// @brief Deleted default constructor. Must provide configuration values.
    ///-----------------------------------------------------------------------------------
    DragonLimelight() = delete;

    ///-----------------------------------------------------------------------------------
    /// @brief Construct a DragonLimelight object.
    /// @details Initializes network table reference, camera pose offsets, pipeline/LED/cam modes,
    ///          and prepares port forwarding for the Limelight. Throws/assumes external code
    ///          provides valid enums and mounting offsets.
    /// @param name NetworkTable name for the Limelight (sanitized internally).
    /// @param identifier Identifier enum for multiple limelights on the robot.
    /// @param cameraType Type hint for camera hardware/configuration.
    /// @param cameraUsage How the camera will be used (odometry, driver camera, etc.).
    /// @param mountingXOffset Forward offset (inches) from robot center.
    /// @param mountingYOffset Left offset (inches) from robot center.
    /// @param mountingZOffset Up offset (inches) from robot center.
    /// @param pitch Camera pitch in degrees.
    /// @param yaw Camera yaw in degrees.
    /// @param roll Camera roll in degrees.
    /// @param initialPipeline Pipeline enum to select on initialization.
    /// @param ledMode Initial LED mode to set.
    /// @param camMode Initial camera mode to set.
    ///-----------------------------------------------------------------------------------
    DragonLimelight(
        std::string name, /// <I> - network table name
        DRAGON_LIMELIGHT_CAMERA_IDENTIFIER identifier,
        DRAGON_LIMELIGHT_CAMERA_TYPE cameraType,
        DRAGON_LIMELIGHT_CAMERA_USAGE cameraUsage,
        units::length::inch_t mountingXOffset,     /// <I> x offset of cam from robot center (forward relative to robot)
        units::length::inch_t mountingYOffset,     /// <I> y offset of cam from robot center (left relative to robot)
        units::length::inch_t mountingZOffset,     /// <I> z offset of cam from robot center (up relative to robot)
        units::angle::degree_t pitch,              /// <I> - Pitch of camera
        units::angle::degree_t yaw,                /// <I> - Yaw of camera
        units::angle::degree_t roll,               /// <I> - Roll of camera
        DRAGON_LIMELIGHT_PIPELINE initialPipeline, /// <I> enum for starting pipeline
        DRAGON_LIMELIGHT_LED_MODE ledMode,
        DRAGON_LIMELIGHT_CAM_MODE camMode);

    ///-----------------------------------------------------------------------------------
    /// @brief Default destructor.
    /// @details Does minimal cleanup (no ownership of heavy resources here). Timer ownership
    ///          is handled in implementation.
    ///-----------------------------------------------------------------------------------
    ~DragonLimelight() = default;

    ///-----------------------------------------------------------------------------------
    /// @brief Get a list of AprilTag detections filtered by validTag (if provided).
    /// @param validTag Optional list of FieldAprilTagIDs; if empty all tags returned.
    /// @return Vector of unique_ptr to DragonVisionStruct, one per valid detection.
    /// @notes Non-blocking; returns empty vector when network table is not available.
    ///-----------------------------------------------------------------------------------
    std::vector<std::unique_ptr<DragonVisionStruct>> GetAprilTagVisionTargetInfo(const std::vector<FieldAprilTagIDs> &validTag = {}) const;

    ///-----------------------------------------------------------------------------------
    /// @brief Get a list of object-detection results filtered by class IDs.
    /// @param validClasses Optional class ID filter; if empty all detections returned.
    /// @return Vector of unique_ptr to DragonVisionStruct containing detection details.
    ///-----------------------------------------------------------------------------------
    std::vector<std::unique_ptr<DragonVisionStruct>> GetObjectDetectionTargetInfo(const std::vector<int> &validClasses = {}) const;

    ///-----------------------------------------------------------------------------------
    /// @brief Check whether the Limelight appears to be running/responding.
    /// @return true if the heartbeat and timer indicate the Limelight is healthy.
    ///-----------------------------------------------------------------------------------
    bool IsLimelightRunning();

    ///-----------------------------------------------------------------------------------
    /// @brief Set the Limelight LED behavior.
    /// @param mode LED mode enum controlling on/off/blink/pipeline control.
    ///-----------------------------------------------------------------------------------
    void SetLEDMode(DRAGON_LIMELIGHT_LED_MODE mode);

    ///-----------------------------------------------------------------------------------
    /// @brief Set the camera mode (vision vs driver camera).
    /// @param mode Camera mode enum to write to the Limelight NT entry.
    ///-----------------------------------------------------------------------------------
    void SetCamMode(DRAGON_LIMELIGHT_CAM_MODE mode);

    ///-----------------------------------------------------------------------------------
    /// @brief Select the active Limelight pipeline index.
    /// @param pipeline Pipeline enum index to set on the Limelight.
    ///-----------------------------------------------------------------------------------
    void SetPipeline(DRAGON_LIMELIGHT_PIPELINE pipeline);

    ///-----------------------------------------------------------------------------------
    /// @brief Get the identifier enum for this camera instance.
    ///-----------------------------------------------------------------------------------
    DRAGON_LIMELIGHT_CAMERA_IDENTIFIER GetCameraIdentifier() { return m_identifier; }

    ///-----------------------------------------------------------------------------------
    /// @brief Request a pose estimate from the Limelight for odometry fusion.
    /// @param useMegatag2 If true use MegaTag2 based estimate, otherwise use MegaTag1.
    /// @return optional VisionPose when valid; std::nullopt when unavailable.
    ///-----------------------------------------------------------------------------------
    std::optional<VisionPose> EstimatePoseOdometryLimelight(bool useMegatag2);

    ///-----------------------------------------------------------------------------------
    /// @brief Retrieve MegaTag1 (standard) pose estimate.
    /// @return optional VisionPose when tags are sufficient and sigma can be computed.
    ///-----------------------------------------------------------------------------------
    std::optional<VisionPose> GetMegaTag1Pose();

    ///-----------------------------------------------------------------------------------
    /// @brief Retrieve MegaTag2 pose estimate (specialized).
    /// @return optional VisionPose when MegaTag2 API provides a valid estimate.
    ///-----------------------------------------------------------------------------------
    std::optional<VisionPose> GetMegaTag2Pose();

    ///-----------------------------------------------------------------------------------
    /// @brief Send a 2D robot pose to the Limelight as an initial orientation hint.
    /// @param pose Pose2d used for initial yaw/orientation in the Limelight fusion.
    ///-----------------------------------------------------------------------------------
    void SetRobotPose(const frc::Pose2d &pose);

    ///-----------------------------------------------------------------------------------
    /// @brief Get sanitized camera name used for network table operations.
    ///-----------------------------------------------------------------------------------
    std::string GetCameraName() const { return m_cameraName; }

private:
    ///-----------------------------------------------------------------------------------
    /// @brief Set the priority AprilTag ID used by Limelight pose selection logic.
    /// @param id Tag id to prioritize.
    ///-----------------------------------------------------------------------------------
    void SetPriorityTagID(int id);

    ///-----------------------------------------------------------------------------------
    /// @brief Publish camera transform relative to robot to the Limelight.
    /// @param forward left/right/up/roll/pitch/yaw in robot-space units (double).
    ///-----------------------------------------------------------------------------------
    void SetCameraPose_RobotSpace(double forward, double left, double up, double roll, double pitch, double yaw);

    ///-----------------------------------------------------------------------------------
    /// @brief IMU mode enum used to configure Limelight's IMU usage.
    ///-----------------------------------------------------------------------------------
    enum class LIMELIGHT_IMU_MODE
    {
        USE_EXTERNAL_IMU_ONLY = 0,
        USE_EXTERNAL_IMU_AND_FUSE_WITH_INTERNAL_IMU,
        USE_INTERNAL_IMU,
        USE_INTERNAL_WITH_MT1_ASSISTED_CONVERGENCE,
        USE_INTERNAL_IMU_WITH_EXTERNAL_IMU_ASSISTED_CONVERGENCE
    };

    ///-----------------------------------------------------------------------------------
    /// @name Member variables
    /// @brief Important internal state used by Limelight wrapper.
    ///-----------------------------------------------------------------------------------
    DRAGON_LIMELIGHT_CAMERA_IDENTIFIER m_identifier;
    std::shared_ptr<nt::NetworkTable> m_limelightNT;

    const double START_HB = -9999;     ///< initial heartbeat sentinel
    const double MAX_HB = 2000000000;  ///< safety max heartbeat (unused currently)
    double m_lastHeartbeat = START_HB; ///< last seen heartbeat value
    frc::Timer *m_healthTimer;         ///< local timer used for heartbeat health checks

    DRAGON_LIMELIGHT_PIPELINE m_pipeline; ///< currently selected pipeline

    // from old dragon camera
    std::string m_cameraName;                            ///< sanitized camera name used with helpers
    subsystems::CommandSwerveDrivetrain *m_chassis;      ///< pointer to chassis for orientation/limits
    frc::Pose3d m_cameraPose;                            ///< camera transform relative to robot
    const double m_maxRotationRateDegreesPerSec = 720.0; ///< fallback limit if chassis not available
    // Small orientation/yaw/pitch members used when setting robot orientation in Limelight
    const double m_yawRate = 0.0;
    const double m_pitch = 0.0;
    const double m_pitchRate = 0.0;
    const double m_roll = 0.0;
    const double m_rollRate = 0.0;

    int m_numberOfTags;             ///< last count of tags reported
    bool m_megatag1PosBool = false; ///< flag: have MegaTag1 estimate
    VisionPose m_megatag1Pos;       ///< last MegaTag1 pose
    bool m_megatag2PosBool = false; ///< flag: have MegaTag2 estimate
    VisionPose m_megatag2Pos;       ///< last MegaTag2 pose
    bool m_robotPoseSet = false;    ///< true after SetRobotPose has been called
};
