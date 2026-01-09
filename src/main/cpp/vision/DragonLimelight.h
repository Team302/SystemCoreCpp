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

// Third Party Includes

// DragonLimelight needs to be a child of DragonCamera
class DragonLimelight
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

    std::vector<std::unique_ptr<DragonVisionStruct>> GetAprilTagVisionTargetInfo(const std::vector<FieldAprilTagIDs> &validTag = {}) const;
    std::vector<std::unique_ptr<DragonVisionStruct>> GetObjectDetectionTargetInfo(const std::vector<int> &validClasses = {}) const;
    bool IsLimelightRunning();

    void SetLEDMode(DRAGON_LIMELIGHT_LED_MODE mode);
    void SetCamMode(DRAGON_LIMELIGHT_CAM_MODE mode);
    void SetPipeline(DRAGON_LIMELIGHT_PIPELINE pipeline);

    DRAGON_LIMELIGHT_CAMERA_IDENTIFIER GetCameraIdentifier() { return m_identifier; }

protected:
    void SetPriorityTagID(int id);
    void SetCameraPose_RobotSpace(double forward, double left, double up, double roll, double pitch, double yaw);

    enum class LIMELIGHT_IMU_MODE
    {
        USE_EXTERNAL_IMU_ONLY = 0,
        USE_EXTERNAL_IMU_AND_FUSE_WITH_INTERNAL_IMU,
        USE_INTERNAL_IMU,
        USE_INTERNAL_WITH_MT1_ASSISTED_CONVERGENCE,
        USE_INTERNAL_IMU_WITH_EXTERNAL_IMU_ASSISTED_CONVERGENCE
    };

private:
    DRAGON_LIMELIGHT_CAMERA_IDENTIFIER m_identifier;
    std::shared_ptr<nt::NetworkTable> m_limelightNT;

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
