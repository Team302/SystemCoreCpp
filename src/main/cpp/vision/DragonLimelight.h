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
#include <string>
#include <vector>

// FRC includes

#include "frc/Timer.h"
#include "networktables/NetworkTable.h"
#include "units/angle.h"
#include "units/length.h"
#include "units/time.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Pose3d.h"
#include "frc/DriverStation.h"

// Team 302 includes
#include "utils/sensors/SensorData.h"
#include "utils/logging/signals/DragonDataLogger.h"
#include "vision/DragonVisionStructs.h"
#include "chassis/pose/DragonVisionPoseEstimator.h"
#include "chassis/ChassisConfigMgr.h"

// Third Party Includes

enum class DRAGON_LIMELIGHT_CAMERA_TYPE
{
    LIMELIGHT4,
    LIMELIGHT4_W_HAILO8,
    LIMELIGHT3G,
    LIMELIGHT3,
    LIMELIGHT3_W_CORAL
};

enum class DRAGON_LIMELIGHT_CAMERA_IDENTIFIER
{
    BACK_CAMERA,
    FRONT_CAMERA
};

enum class DRAGON_LIMELIGHT_CAMERA_USAGE
{
    APRIL_TAGS,
    OBJECT_DETECTION_ALGAE,
    ALGAE_AND_APRIL_TAGS
};

enum class DRAGON_LIMELIGHT_LED_MODE
{
    LED_UNKNOWN = -1,
    LED_PIPELINE_CONTROL,
    LED_OFF,
    LED_BLINK,
    LED_ON
};

enum class DRAGON_LIMELIGHT_CAM_MODE
{
    CAM_UNKNOWN = -1,
    CAM_VISION,
    CAM_DRIVER
};

enum class DRAGON_LIMELIGHT_STREAM_MODE
{
    STREAM_UNKNOWN = -1,
    STREAM_STANDARD,     // side by side if two cams
    STREAM_PIP_MAIN,     // Second Cam bottom right of Main Cam
    STREAM_PIP_SECONDARY // Main Cam bottom right of Second Cam
};

enum class DRAGON_LIMELIGHT_SNAPSHOT_MODE
{
    SNAPSHOT_MODE_UNKNOWN = -1,
    SNAP_OFF,
    SNAP_ON
};

enum class DRAGON_LIMELIGHT_PIPELINE
{
    UNKNOWN = -1,
    APRIL_TAG = 0,
    MACHINE_LEARNING_PL = 1,
    COLOR_THRESHOLD
};

// DragonLimelight needs to be a child of DragonCamera
class DragonLimelight : public DragonVisionPoseEstimator, public SensorData, public DragonDataLogger
{
public:
    ///-----------------------------------------------------------------------------------
    /// Method:         DragonLimelight (constructor)
    /// Description:    Create the object
    ///-----------------------------------------------------------------------------------
    DragonLimelight() = delete;
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
    /// Method:         ~DragonLimelight (destructor)
    /// Description:    Delete the object
    ///-----------------------------------------------------------------------------------
    ~DragonLimelight() = default;

    bool HealthCheck() override;
    bool HasTarget();

    virtual std::optional<units::angle::degree_t> GetTargetYaw();
    std::optional<units::angle::degree_t> GetTargetYawRobotFrame();
    virtual std::optional<units::angle::degree_t> GetTargetPitch();
    std::optional<units::angle::degree_t> GetTargetPitchRobotFrame();
    std::optional<double> GetTargetArea();
    std::optional<units::angle::degree_t> GetTargetSkew();
    std::optional<units::time::millisecond_t> GetPipelineLatency();
    std::optional<int> GetAprilTagID();

    std::optional<VisionPose> EstimatePoseOdometryLimelight(bool megatag2);

    std::optional<VisionData> GetDataToNearestAprilTag();
    std::optional<VisionData> GetDataToSpecifiedTag(int id);

    std::optional<units::length::inch_t> EstimateTargetXDistance();
    std::optional<units::length::inch_t> EstimateTargetYDistance();
    std::optional<units::length::inch_t> EstimateTargetZDistance();

    std::optional<units::length::inch_t> EstimateTargetXDistance_RelToRobotCoords();
    std::optional<units::length::inch_t> EstimateTargetYDistance_RelToRobotCoords();
    std::optional<units::length::inch_t> EstimateTargetZDistance_RelToRobotCoords();

    units::length::inch_t CalcXTargetToRobot(units::angle::degree_t camPitch, units::length::inch_t mountHeight, units::length::inch_t camXOffset, units::angle::degree_t tY);
    units::length::inch_t CalcYTargetToRobot(units::angle::degree_t camYaw, units::length::inch_t xTargetDistance, units::length::inch_t camYOffset, units::length::inch_t camXOffset, units::angle::degree_t tX);

    // limelight specific helper functions
    void SetLEDMode(DRAGON_LIMELIGHT_LED_MODE mode);
    void SetCamMode(DRAGON_LIMELIGHT_CAM_MODE mode);
    void SetPipeline(DRAGON_LIMELIGHT_PIPELINE pipeline);
    void SetPriorityTagID(int id);
    void SetCameraPose_RobotSpace(double forward, double left, double up, double roll, double pitch, double yaw);

    DRAGON_LIMELIGHT_PIPELINE GetPipeline() const { return m_pipeline; }
    units::angle::degree_t GetCameraPitch() const { return m_cameraPose.Rotation().Y(); }
    units::angle::degree_t GetCameraYaw() const { return m_cameraPose.Rotation().Z(); }
    units::angle::degree_t GetCameraRoll() const { return m_cameraPose.Rotation().X(); } // rotates around x-axis
    units::length::inch_t GetMountingXOffset() const { return m_cameraPose.X(); }
    units::length::inch_t GetMountingYOffset() const { return m_cameraPose.Y(); }
    units::length::inch_t GetMountingZOffset() const { return m_cameraPose.Z(); }
    std::string GetCameraName() const { return m_cameraName; }
    DRAGON_LIMELIGHT_CAMERA_IDENTIFIER GetCameraIdentifier() { return m_identifier; }

    void PeriodicCacheData() override;

    units::angle::degree_t GetTx() const;
    units::angle::degree_t GetTy() const;

    void PrintValues(); // Prints out all values to ensure everything is working and connected

    DragonVisionPoseEstimatorStruct GetPoseEstimate() override;
    void DataLog(uint64_t timestamp) override;
    void SetRobotPose(const frc::Pose2d &pose) override;

protected:
    enum class LIMELIGHT_IMU_MODE
    {
        USE_EXTERNAL_IMU_ONLY = 0,
        USE_EXTERNAL_IMU_AND_FUSE_WITH_INTERNAL_IMU,
        USE_INTERNAL_IMU,
        USE_INTERNAL_WITH_MT1_ASSISTED_CONVERGENCE,
        USE_INTERNAL_IMU_WITH_EXTERNAL_IMU_ASSISTED_CONVERGENCE
    };
    units::length::inch_t m_driveThroughOffset = units::length::inch_t(0.0);

    DRAGON_LIMELIGHT_CAMERA_IDENTIFIER m_identifier;
    std::shared_ptr<nt::NetworkTable> m_networktable;

    // cached elements
    bool m_tv;
    units::angle::degree_t m_tx;
    units::angle::degree_t m_ty;
    int m_tagid;
    bool m_megatag2PosBool = false;
    bool m_megatag1PosBool = false;
    VisionPose m_megatag2Pos;
    VisionPose m_megatag1Pos;

    const double START_HB = -9999;
    const double MAX_HB = 2000000000;
    double m_lastHeartbeat = START_HB;
    frc::Timer *m_healthTimer;
    DRAGON_LIMELIGHT_PIPELINE m_pipeline;

    // from old dragon camera
    std::string m_cameraName;
    subsystems::CommandSwerveDrivetrain *m_chassis;
    frc::Pose3d m_cameraPose;
    const double m_maxRotationRateDegreesPerSec = 720.0;
    const double m_yawRate = 0.0;
    const double m_pitch = 0.0;
    const double m_pitchRate = 0.0;
    const double m_roll = 0.0;
    const double m_rollRate = 0.0;
    int m_numberOfTags;
};
