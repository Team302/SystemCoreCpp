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
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "units/angle.h"
#include "units/length.h"
#include "units/time.h"
#include "networktables/DoubleArrayTopic.h"
#include "frc/DriverStation.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/Timer.h"
#include "units/length.h"
#include "units/time.h"
#include "frc/RobotBase.h"

// Team 302 includes
#include "chassis/ChassisConfigMgr.h"
#include "chassis/pose/DragonSwervePoseEstimator.h"
#include "vision/DragonLimelight.h"
#include "utils/logging/debug/Logger.h"
#include "vision/DragonVision.h"
#include "vision/DragonVisionStructLogger.h"

// Third Party Includes
#include "Limelight/LimelightHelpers.h"

/// TODO
/// Need to support DragonLimelight becoming a child of DragonCamera
/// Need to remove everything involving target height, should use apriltag field positions
/// Need to support new OriginFieldPosition function, look at limelight docs NetworkTables API

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
    DRAGON_LIMELIGHT_CAM_MODE camMode) : DragonVisionPoseEstimator(),
                                         SensorData(),
                                         DragonDataLogger(),
                                         m_identifier(identifier),
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

void DragonLimelight::PeriodicCacheData()
{
    m_megatag1PosBool = false;
    m_megatag2PosBool = false;

    m_tv = LimelightHelpers::getTV(m_cameraName);
    m_tx = units::angle::degree_t(LimelightHelpers::getTX(m_cameraName));
    m_ty = units::angle::degree_t(LimelightHelpers::getTY(m_cameraName));
    m_tagid = LimelightHelpers::getFiducialID(m_cameraName);
    m_pipeline = static_cast<DRAGON_LIMELIGHT_PIPELINE>(LimelightHelpers::getCurrentPipelineIndex(m_cameraName));
}

bool DragonLimelight::HealthCheck()
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

/// @brief Assume that the current pipeline is AprilTag and that a target is detected
/// @return -1 if the network table cannot be found
std::optional<int> DragonLimelight::GetAprilTagID()
{
    return m_tagid;
}

bool DragonLimelight::HasTarget()
{
    return m_tv;
}

units::angle::degree_t DragonLimelight::GetTx() const
{
    return m_tx;
}

units::angle::degree_t DragonLimelight::GetTy() const
{
    return m_ty;
}

std::optional<units::angle::degree_t> DragonLimelight::GetTargetYaw()
{
    return -1.0 * GetTx();
}

std::optional<units::angle::degree_t> DragonLimelight::GetTargetYawRobotFrame()
{
    std::optional<units::length::inch_t> targetXdistance = EstimateTargetXDistance_RelToRobotCoords();
    std::optional<units::length::inch_t> targetYdistance = EstimateTargetYDistance_RelToRobotCoords();

    if (targetXdistance.has_value() && targetYdistance.has_value())
    {
        if (std::abs(targetXdistance.value().to<double>()) > 0)
        {
            return units::math::atan2(targetYdistance.value(), targetXdistance.value());
        }
        else
        {
            return units::angle::degree_t(0.0);
        }
    }

    return std::nullopt;
}

std::optional<units::angle::degree_t> DragonLimelight::GetTargetPitch()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, std::string("DragonLimelight"), std::string("GetTargetVerticalOffset"), std::string("Invalid limelight rotation"));
    return GetTy();
}

std::optional<units::angle::degree_t> DragonLimelight::GetTargetPitchRobotFrame()
{
    std::optional<units::length::inch_t> targetXDistance = std::optional<units::length::inch_t>(EstimateTargetXDistance_RelToRobotCoords());
    std::optional<units::length::inch_t> targetZDistance = EstimateTargetZDistance_RelToRobotCoords();

    if (targetXDistance && targetZDistance)
    {
        units::angle::degree_t targetPitchToRobot = units::angle::degree_t(atan2(targetZDistance.value().to<double>(), targetXDistance.value().to<double>()));
        return targetPitchToRobot;
    }

    return std::nullopt;
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

/**
 * @brief Get the Pose object for the current location of the robot
 * https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization
 */
std::optional<VisionPose> DragonLimelight::EstimatePoseOdometryLimelight(bool megatag2)
{
    if (frc::RobotBase::IsSimulation())
    {
        return std::nullopt;
    }
    // use megatag1
    // megatag2 = false;  TODO: Try Again without this
    auto mode = static_cast<int>(LIMELIGHT_IMU_MODE::USE_EXTERNAL_IMU_ONLY); // Chief Delphi answer says perfect portrait pose doesn't work with internal IMU

    // Megatag 1
    if (m_networktable.get() != nullptr)
    {
        LimelightHelpers::SetIMUMode(m_cameraName, mode);
        // Megatag 1
        if (!megatag2)
        {
            if (!m_megatag1PosBool)
            {
                nt::DoubleArrayTopic topic = m_networktable.get()->GetDoubleArrayTopic("botpose_wpiblue");
                std::vector<double> position = topic.GetEntry(std::array<double, 7>{}).Get(); // default value is empty array

                units::time::millisecond_t currentTime = frc::Timer::GetFPGATimestamp();
                units::time::millisecond_t timestamp = currentTime - units::millisecond_t(position[6] / 1000.0);

                frc::Rotation3d rotation = frc::Rotation3d{units::angle::degree_t(position[3]), units::angle::degree_t(position[4]), units::angle::degree_t(position[5])};
                frc::Pose3d pose3d = frc::Pose3d{units::meter_t(position[0]), units::meter_t(position[1]), units::meter_t(position[2]), rotation};

                double numberOfTagsDetected = position[7];
                double averageTagTargetArea = position[10];

                // in case of invalid Limelight targets
                if (pose3d.ToPose2d().X() == units::meter_t(0.0))
                {
                    return std::nullopt;
                }

                double xyStds;
                double degStds;
                // multiple targets detected
                if (numberOfTagsDetected == 0)
                {
                    return std::nullopt;
                }
                else if (numberOfTagsDetected >= 2)
                {
                    xyStds = 0.5;
                    degStds = 6;
                }
                // 1 target with large area (close to camera))
                else if (averageTagTargetArea > 0.8)
                {
                    xyStds = 1.0;
                    degStds = 12;
                }
                // TODO: tune this!
                // 1 target with medium area (somewhat close to camera))
                // else if (averageTagTargetArea > 0.45)
                // {
                //     xyStds = 1.5;
                //     degStds = 21;
                // }
                // 1 target small area (farther away from camera)
                else if (averageTagTargetArea > 0.1)
                {
                    xyStds = 2.0;
                    degStds = 30;
                }
                // conditions don't match to add a vision measurement
                else
                {
                    return std::nullopt;
                }
                m_megatag1PosBool = true;
                m_megatag1Pos = {pose3d, timestamp, {xyStds, xyStds, degStds}, PoseEstimationStrategy::MEGA_TAG};
            }
            return m_megatag1Pos;
        }

        // Megatag 2
        else
        {
            if (!m_megatag2PosBool)
            {
                // auto mode = frc::DriverStation::IsDisabled() ? static_cast<int>(LIMELIGHT_IMU_MODE::USE_EXTERNAL_IMU_AND_FUSE_WITH_INTERNAL_IMU) : static_cast<int>(LIMELIGHT_IMU_MODE::USE_INTERNAL_IMU);
                LimelightHelpers::SetIMUMode(m_cameraName, mode);
                auto poseEstimate = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(m_cameraName);

                // multiple targets detected
                if (poseEstimate.tagCount == 0)
                {
                    return std::nullopt;
                }
                // conditions don't match to add a vision measurement
                else
                {
                    double xyStds = .7;
                    double degStds = 9999999;
                    m_megatag2PosBool = true;
                    m_megatag2Pos = {frc::Pose3d{poseEstimate.pose}, poseEstimate.timestampSeconds, {xyStds, xyStds, degStds}, PoseEstimationStrategy::MEGA_TAG_2};
                    // auto mode = frc::DriverStation::IsDisabled() ? static_cast<int>(LIMELIGHT_IMU_MODE::USE_EXTERNAL_IMU_AND_FUSE_WITH_INTERNAL_IMU) : static_cast<int>(LIMELIGHT_IMU_MODE::USE_INTERNAL_IMU);
                    LimelightHelpers::SetIMUMode(m_cameraName, mode);
                }
            }
            return m_megatag2Pos;
        }
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

units::length::inch_t DragonLimelight::CalcXTargetToRobot(units::angle::degree_t camPitch, units::length::inch_t mountHeight, units::length::inch_t camXOffset, units::angle::degree_t tY)
{
    units::length::inch_t XDistance = units::length::inch_t((units::math::tan(units::angle::degree_t(90) + camPitch + tY) * mountHeight) + units::math::abs(camXOffset));

    if (GetCameraYaw() > units::degree_t(std::abs(90.0)))
    {
        return -1.0 * (XDistance + m_driveThroughOffset);
    }
    return XDistance + m_driveThroughOffset;
}

units::length::inch_t DragonLimelight::CalcYTargetToRobot(units::angle::degree_t camYaw, units::length::inch_t xTargetDistance, units::length::inch_t camYOffset, units::length::inch_t camXOffset, units::angle::degree_t tX)
{
    units::length::inch_t yDistance = units::length::inch_t(0.0);
    if (GetCameraYaw() > units::degree_t(std::abs(90.0)))
        yDistance = units::length::inch_t((units::math::tan(tX + camYaw) * (xTargetDistance - camXOffset)) + camYOffset);
    else
        yDistance = units::length::inch_t((units::math::tan(tX + camYaw) * (xTargetDistance - camXOffset)) - camYOffset);

    return yDistance;
}

std::optional<units::length::inch_t> DragonLimelight::EstimateTargetXDistance()
{
    units::length::meter_t mountingHeight = m_cameraPose.Z();

    units::angle::degree_t mountingAngle = m_cameraPose.Rotation().Y() * -1;
    std::optional<units::angle::degree_t> targetPitch = GetTargetPitch();
    std::optional<int> aprilTagID = GetAprilTagID();
    if (!aprilTagID)
    {
        if (targetPitch)
        {
            double tangent = units::math::tan(mountingAngle + targetPitch.value());
            if (tangent == 0)
            {
                return std::nullopt;
            }
            else
            {
                // TODO come back to this with different math
                // units::length::inch_t estimatedTargetDistance = (m_noteVerticalOffset - mountingHeight) / tangent;
                units::length::inch_t estimatedTargetDistance = (mountingHeight) / tangent;

                return estimatedTargetDistance;
            }
        }
    }
    else
    {
        auto botpose = m_networktable.get()->GetDoubleArrayTopic("targetpose_robotspace");
        std::vector<double> xdistance = botpose.GetEntry(std::array<double, 6>{}).Get(); // default value is empty array

        return units::length::inch_t(xdistance[0]);
    }

    return std::nullopt;
}

std::optional<units::length::inch_t> DragonLimelight::EstimateTargetYDistance()
{
    std::optional<int> aprilTagID = GetAprilTagID();
    std::optional<units::angle::degree_t> targetYaw = GetTargetYaw();
    std::optional<units::length::inch_t> targetXdistance = EstimateTargetXDistance();
    if (!aprilTagID && targetYaw && targetXdistance)
    {
        units::length::inch_t estimatedTargetDistance = targetXdistance.value() * units::math::tan(m_cameraPose.Rotation().Z() + targetYaw.value());
        return estimatedTargetDistance;
    }
    else if (aprilTagID)
    {
        auto botpose = m_networktable.get()->GetDoubleArrayTopic("targetpose_robotspace");
        std::vector<double> xdistance = botpose.GetEntry(std::array<double, 6>{}).Get(); // default value is empty array

        return units::length::inch_t(xdistance[1]);
    }

    return std::nullopt;
}

std::optional<units::length::inch_t> DragonLimelight::EstimateTargetZDistance()
{

    if (!GetAprilTagID())
    {
        // TODO COME BACK TO THIS ONE
        // units::length::inch_t estimatedTargetZDistance = m_cameraPose.Z() - m_noteVerticalOffset;
        units::length::inch_t estimatedTargetZDistance = m_cameraPose.Z();
        return estimatedTargetZDistance;
    }

    else
    {
        auto botpose = m_networktable.get()->GetDoubleArrayTopic("targetpose_robotspace");
        std::vector<double> xdistance = botpose.GetEntry(std::array<double, 6>{}).Get(); // default value is empty array

        return units::length::inch_t(xdistance[2]);
    }

    return std::nullopt;
}

std::optional<units::length::inch_t> DragonLimelight::EstimateTargetXDistance_RelToRobotCoords()
{
    units::angle::degree_t camPitch = GetCameraPitch();
    units::length::inch_t mountHeight = GetMountingZOffset();
    units::length::inch_t camXOffset = GetMountingXOffset();
    units::angle::degree_t Ty = GetTargetPitch().value();
    return CalcXTargetToRobot(camPitch, mountHeight, camXOffset, Ty);
}

std::optional<units::length::inch_t> DragonLimelight::EstimateTargetYDistance_RelToRobotCoords()
{

    units::angle::degree_t camYaw = GetCameraYaw();
    units::length::inch_t camYOffset = GetMountingYOffset();
    units::length::inch_t camXOffset = GetMountingXOffset();
    units::angle::degree_t Tx = GetTargetYaw().value();
    units::length::inch_t xTargetDistance = CalcXTargetToRobot(GetCameraPitch(), GetMountingZOffset(), GetMountingXOffset(), GetTargetPitch().value());

    return CalcYTargetToRobot(camYaw, xTargetDistance, camYOffset, camXOffset, Tx);
}

std::optional<units::length::inch_t> DragonLimelight::EstimateTargetZDistance_RelToRobotCoords()
{
    std::optional<units::length::inch_t> zDistance = EstimateTargetZDistance();
    if (zDistance)
    {
        units::length::inch_t targetZoffset_RF_inch = zDistance.value() + GetMountingZOffset(); ///< the offset is positive if the limelight is above the center of the robot

        return targetZoffset_RF_inch;
    }

    return std::nullopt;
}

std::optional<VisionData> DragonLimelight::GetDataToNearestAprilTag()
{
    std::optional<int> tagId = GetAprilTagID();
    if (tagId)
    {
        auto targetPose = m_networktable.get()->GetDoubleArrayTopic("targetpose_robotspace");

        std::vector<double> vector = targetPose.GetEntry(std::array<double, 6>{}).Get();

        frc::Rotation3d rotation = frc::Rotation3d(units::angle::degree_t(vector[5]), units::angle::degree_t(vector[3]), units::angle::degree_t(vector[4]));
        auto transform = frc::Transform3d(units::length::meter_t(vector[0]), units::length::meter_t(vector[1]), units::length::meter_t(vector[2]), rotation);

        return VisionData{
            transform, transform.Translation(), rotation, tagId.value()};
    }

    return std::nullopt;
}

std::optional<VisionData> DragonLimelight::GetDataToSpecifiedTag(int id)
{
    std::optional<int> detectedTag = GetAprilTagID();
    if (detectedTag.has_value())
    {
        if (detectedTag.value() == id)
        {
            auto targetPose = m_networktable.get()->GetDoubleArrayTopic("targetpose_robotspace");

            std::vector<double> vector = targetPose.GetEntry(std::array<double, 6>{}).Get();

            // targetpose_robotspace: 3D transform of the primary in-view AprilTag in the coordinate system of the Robot (array (6)) [tx, ty, tz, pitch, yaw, roll] (meters, degrees)
            frc::Rotation3d rotationTransform = frc::Rotation3d(units::angle::degree_t(vector[5]), units::angle::degree_t(vector[3]), -units::angle::degree_t(vector[4]));
            auto transform = frc::Transform3d(units::length::meter_t(vector[0]), units::length::meter_t(vector[1]), units::length::meter_t(vector[2]), rotationTransform);

            frc::Rotation3d rotationToTarget = frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::math::atan2(transform.X(), transform.Z())); // roll pitch yaw

            return VisionData{transform, transform.Translation(), rotationToTarget, detectedTag.value()};
        }
    }

    return std::nullopt;
}

DragonVisionPoseEstimatorStruct DragonLimelight::GetPoseEstimate()
{
    if (m_chassis != nullptr && ChassisConfigMgr::GetInstance()->GetRotationRateDegreesPerSecond() < m_maxRotationRateDegreesPerSec)
    {

        LimelightHelpers::SetRobotOrientation(GetCameraName(),
                                              m_chassis->GetPose().Rotation().Degrees().value(),
                                              m_yawRate,
                                              m_pitch,
                                              m_pitchRate,
                                              m_roll,
                                              m_rollRate);

        std::optional<VisionPose> megaTag2Pose = EstimatePoseOdometryLimelight(true);

        if (megaTag2Pose.has_value())
        {
            if (EstimateTargetXDistance())
            {
                DragonVisionPoseEstimatorStruct str;
                if (EstimateTargetXDistance().value().to<double>() < 36)
                {
                    str.m_confidenceLevel = DragonVisionPoseEstimatorStruct::ConfidenceLevel::HIGH;
                }
                else
                {
                    str.m_confidenceLevel = DragonVisionPoseEstimatorStruct::ConfidenceLevel::MEDIUM;
                }
                str.m_stds = megaTag2Pose.value().visionMeasurementStdDevs;
                str.m_timeStamp = megaTag2Pose.value().timeStamp;
                str.m_visionPose = megaTag2Pose.value().estimatedPose.ToPose2d();
                return str;
            }
        }
    }
    return DragonVisionPoseEstimatorStruct();
}

void DragonLimelight::DataLog(uint64_t timestamp)
{
    auto vispose = EstimatePoseOdometryLimelight(true);
    if (vispose.has_value())
    {
        if (m_identifier == DRAGON_LIMELIGHT_CAMERA_IDENTIFIER::FRONT_CAMERA)
        {
            Log3DPoseData(timestamp, DragonDataLogger::PoseSingals::CURRENT_CHASSIS_LIMELIGHT_POSE3D, vispose.value().estimatedPose);
        }
        else
        {
            Log3DPoseData(timestamp, DragonDataLogger::PoseSingals::CURRENT_CHASSIS_LIMELIGHT2_POSE3D, vispose.value().estimatedPose);
        }
    }

    DragonDataLogger::LogDoubleData(timestamp, DragonDataLogger::DoubleSignals::LIMELIGHT1_NUMBER_OF_TAGS, m_numberOfTags);
    DragonDataLogger::LogBoolData(timestamp, DragonDataLogger::BoolSignals::IS_ALGAE_DETECTED, m_tv);
}

void DragonLimelight::SetRobotPose(const frc::Pose2d &pose)
{
    return;
    auto yawrate = 0.0;
    auto pitch = 0.0;
    auto pitchrate = 0.0;
    auto roll = 0.0;
    auto rollrate = 0.0;
    if (m_chassis != nullptr)
    {
        yawrate = ChassisConfigMgr::GetInstance()->GetRotationRateDegreesPerSecond();
        pitch = GetCameraPitch().value();
        roll = GetCameraRoll().value();
    }

    LimelightHelpers::SetRobotOrientation(m_cameraName,
                                          pose.Rotation().Degrees().value(),
                                          yawrate,
                                          pitch,
                                          pitchrate,
                                          roll,
                                          rollrate);
}
