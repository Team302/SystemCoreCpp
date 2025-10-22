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
                                         m_networktable(nt::NetworkTableInstance::GetDefault().GetTable(std::string(networkTableName))),
                                         m_chassis(ChassisConfigMgr::GetInstance()->GetSwerveChassis()),
                                         m_cameraPose(frc::Pose3d(mountingXOffset, mountingYOffset, mountingZOffset, frc::Rotation3d(roll, pitch, yaw)))
{
    SetLEDMode(ledMode);
    SetCamMode(camMode);
    SetPipeline(initialPipeline);
    SetCameraPose_RobotSpace(mountingXOffset.to<double>(), mountingYOffset.to<double>(), mountingZOffset.to<double>(), roll.to<double>(), pitch.to<double>(), yaw.to<double>());
    m_cameraName = networkTableName;
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

    auto nt = m_networktable.get();
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

std::vector<std::unique_ptr<DragonVisionStruct>> DragonLimelight::GetVisionTargetInfo(
    VisionTargetOption option,
    DragonTargetType targetType,
    std::vector<FieldAprilTagIDs> validAprilTagIDs)
{
    std::vector<std::unique_ptr<DragonVisionStruct>> targets;

    if (frc::RobotBase::IsSimulation())
    {
        return targets; // In simulation, we don't have a limelight, so just return empty vector
    }

    if (targetType == DragonTargetType::APRIL_TAG)
    {
        targets = ProcessAprilTags(option, validAprilTagIDs);
    }
    else if (targetType == DragonTargetType::AGLAE)
    {
        targets = ProcessAlgae(option);
    }

    return targets;
}

std::vector<std::unique_ptr<DragonVisionStruct>> DragonLimelight::ProcessAprilTags(
    VisionTargetOption option,
    std::vector<FieldAprilTagIDs> validAprilTagIDs)
{
    std::vector<std::unique_ptr<DragonVisionStruct>> targets;
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        // auto aprilTags = LimelightHelpers::getRawHelpers(m_cameraName);

        auto tagID = GetAprilTagID();
        if (tagID.has_value())
        {
            auto aprilTagValue = std::make_unique<DragonVisionStruct>();
            aprilTagValue.get()->targetID = tagID.value();
            aprilTagValue.get()->targetType = DragonTargetType::APRIL_TAG;
            aprilTagValue.get()->className = "AprilTag";
            aprilTagValue.get()->horizontalOffset = GetTx();
            aprilTagValue.get()->verticalOffset = GetTy();
            aprilTagValue.get()->targetArea = nt->GetNumber("ta", 0.0);
            aprilTagValue.get()->pipelineLatency = units::millisecond_t(LimelightHelpers::getLatency_Pipeline(m_cameraName) + LimelightHelpers::getLatency_Capture(m_cameraName));
            // aprilTagValue.get()->distanceToCamera = EstimateTargetXDistance().value_or(units::length::meter_t(0.0));
            // aprilTagValue.get()->distanceToRobot = EstimateTargetXDistance_RelToRobotCoords().value_or(units::length::meter_t(0.0));
            aprilTagValue.get()->distanceToCamera = units::length::meter_t(0.0); // TODO: Get Values
            aprilTagValue.get()->distanceToRobot = units::length::meter_t(0.0);  // TODO: Get Values
            aprilTagValue.get()->ambiguity = 0.0;                                // TODO: only april tags
        }
    }
    // aprilTagValue.get()->targetID = GetAprilTagID();

    // int targetID;
    // DragonTargetType targetType;
    // std::string className; // only used by Machine Learning
    // units::angle::degree_t horizontalOffset;
    // units::angle::degree_t verticalOffset;
    // double targetArea;
    // units::time::millisecond_t pipelineLatency; // should be tl + cl
    // units::length::meter_t distanceToCamera;
    // units::length::meter_t distanceToRobot;
    // double ambiguity; // only april tags

    return targets;
}

std::vector<std::unique_ptr<DragonVisionStruct>> DragonLimelight::ProcessAlgae(VisionTargetOption option)
{
    std::vector<std::unique_ptr<DragonVisionStruct>> targets;

    return targets;
}

/// @brief Assume that the current pipeline is AprilTag and that a target is detected
/// @return -1 if the network table cannot be found
std::optional<int> DragonLimelight::GetAprilTagID()
{
    return LimelightHelpers::getFiducialID(m_cameraName);
}

bool DragonLimelight::HasTarget()
{
    return LimelightHelpers::getTV(m_cameraName);
}

units::angle::degree_t DragonLimelight::GetTx() const
{
    return units::angle::degree_t(LimelightHelpers::getTX(m_cameraName));
}

units::angle::degree_t DragonLimelight::GetTy() const
{
    return units::angle::degree_t(LimelightHelpers::getTY(m_cameraName));
}

std::optional<units::angle::degree_t> DragonLimelight::GetTargetYaw()
{
    return -1.0 * GetTx();
}

std::optional<double> DragonLimelight::GetTargetArea()
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return nt->GetNumber("ta", 0.0);
    }

    return std::nullopt;
}

std::optional<units::angle::degree_t> DragonLimelight::GetTargetSkew()
{
    if (m_networktable != nullptr)
    {
        return units::angle::degree_t(m_networktable->GetNumber("ts", 0.0));
    }

    return std::nullopt;
}

std::optional<units::time::millisecond_t> DragonLimelight::GetPipelineLatency()
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return units::time::second_t(LimelightHelpers::getLatency_Pipeline(m_cameraName));
    }

    return std::nullopt;
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
    auto nt = m_networktable.get();
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

void DragonLimelight::PrintValues()
{ /*
    Should do something similar but in DragonCamera instead of DragonLimelight

     Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("PrintValues HasTarget"), to_string(HasTarget()));
     Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("PrintValues XOffset"), to_string(GetTargetHorizontalOffset().to<double>()));
     Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("PrintValues YOffset"), to_string(GetTargetVerticalOffset().to<double>()));
     Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("PrintValues Area"), to_string(GetTargetArea()));
     Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("PrintValues Skew"), to_string(GetTargetSkew().to<double>()));
     Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("PrintValues Latency"), to_string(GetPipelineLatency().to<double>()));
 */
}
