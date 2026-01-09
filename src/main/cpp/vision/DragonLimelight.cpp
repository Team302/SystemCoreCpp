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
#include "chassis/pose/DragonSwervePoseEstimator.h"
#include "utils/logging/debug/Logger.h"
#include "vision/DragonLimelight.h"
#include "vision/DragonVision.h"

// Third Party Includes
#include "Limelight/LimelightHelpers.h"

namespace
{
    bool IsValidAprilTag(const std::vector<FieldAprilTagIDs> &validTags, int tagID);
    bool IsValidObjectClass(const std::vector<int> &validClasses, int classID);
}

///-----------------------------------------------------------------------------------
/// Method:         DragonLimelight (constructor)
/// Description:    Create the object
///-----------------------------------------------------------------------------------
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
            objectValue.get()->objectDetectionData.mountingXOffset = m_cameraPose.X();
            objectValue.get()->objectDetectionData.mountingYOffset = m_cameraPose.Y();
            objectValue.get()->objectDetectionData.mountingZOffset = m_cameraPose.Z();
            objectValue.get()->objectDetectionData.camPitch = m_cameraPose.Rotation().Y();
            objectValue.get()->objectDetectionData.camYaw = m_cameraPose.Rotation().Z();
            objectValue.get()->objectDetectionData.camRoll = m_cameraPose.Rotation().X();

            targets.emplace_back(std::move(objectValue));
        }
    }

    return targets;
}

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
void DragonLimelight::SetPipeline(DRAGON_LIMELIGHT_PIPELINE pipeline)
{
    m_pipeline = pipeline;
    LimelightHelpers::setPipelineIndex(m_cameraName, static_cast<int>(pipeline));
}

void DragonLimelight::SetPriorityTagID(int id)
{
    LimelightHelpers::setPriorityTagID(m_cameraName, id);
}

void DragonLimelight::SetCameraPose_RobotSpace(double forward, double left, double up, double roll, double pitch, double yaw)
{
    LimelightHelpers::setCameraPose_RobotSpace(m_cameraName, forward, left, up, roll, pitch, yaw);
}

namespace
{
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
}
