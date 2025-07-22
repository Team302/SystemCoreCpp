
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

#include "ctre/phoenix6/SignalLogger.hpp"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "utils/logging/signals/DragonDataLogger.h"
#include "utils/logging/signals/DragonDataLoggerMgr.h"

using ctre::phoenix6::SignalLogger;

DragonDataLogger::DragonDataLogger()
{
    DragonDataLoggerMgr::GetInstance()->RegisterItem(this);
}

void DragonDataLogger::LogBoolData(uint64_t timestamp, DragonDataLogger::BoolSignals signalID, bool value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr != nullptr)
    {
        switch (signalID)
        {
        case DragonDataLogger::BoolSignals::IS_BROWNOUT:
            SignalLogger::WriteBoolean(m_brownOutPath, value, m_latency);
            break;
        case DragonDataLogger::DRIVE_TO_IS_DONE:
            SignalLogger::WriteBoolean(m_IsDonePath, value, m_latency);
            break;
        case DragonDataLogger::BoolSignals::IS_ALGAE_DETECTED:
            SignalLogger::WriteBoolean(m_IsAlgaeDetected, value, m_latency);
            break;
        default:
            break;
        }
    }
}

void DragonDataLogger::LogDoubleData(uint64_t timestamp, DragonDataLogger::DoubleSignals signalID, double value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr != nullptr)
    {
        switch (signalID)
        {
        case DragonDataLogger::DoubleSignals::CHASSIS_STORED_HEADING_DEGREES:
            SignalLogger::WriteDouble(m_storedHeadingPath, value, m_storedHeadingUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::CHASSIS_YAW_DEGREES:
            SignalLogger::WriteDouble(m_ChassisYawPath, value, m_ChassisYawUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::ELECTRICAL_VOLTAGE:
            SignalLogger::WriteDouble(m_electricalVoltagePath, value, m_electricalVoltageUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::ELECTRICAL_POWER:
            SignalLogger::WriteDouble(m_electricalPowerPath, value, m_electricalCurrentUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::ELECTRICAL_CURRENT:
            SignalLogger::WriteDouble(m_electricalCurrentPath, value, m_electricalCurrentUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::LIMELIGHT_TV_1:
            SignalLogger::WriteDouble(m_tvPath, value, m_tvUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::LIMELIGHT_TX_1:
            SignalLogger::WriteDouble(m_txPath, value, m_txUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::LIMELIGHT_TY_1:
            SignalLogger::WriteDouble(m_tyPath, value, m_tyUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::LIMELIGHT_FIDUCIAL_ID_1:
            SignalLogger::WriteDouble(m_fiducialPath, value, m_fiducialUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::BATTERY_VOLTAGE:
            SignalLogger::WriteDouble(m_batteryVoltagePath, value, m_batteryVoltageUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::BROWNOUT_VOLTAGE:
            SignalLogger::WriteDouble(m_brownoutVoltagePath, value, m_brownoutVoltageUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::INPUT_VOLTAGE:
            SignalLogger::WriteDouble(m_inputVoltagePath, value, m_inputVoltageUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::INPUT_CURRENT:
            SignalLogger::WriteDouble(m_inputCurrentPath, value, m_inputCurrentUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::CPU_TEMP:
            SignalLogger::WriteDouble(m_cpuTempPath, value, m_cpuTempUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_STEER_POWER:
            SignalLogger::WriteDouble(m_lfSteerPowerPath, value, m_lfSteerPowerUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_STEER_CURRENT:
            SignalLogger::WriteDouble(m_lfSteerCurrentPath, value, m_lfSteerCurrentUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_STEER_TOTALPOWER:
            SignalLogger::WriteDouble(m_lfSteerTotalPowerPath, value, m_lfSteerTotalPowerUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_STEER_WATT_HOURS:
            SignalLogger::WriteDouble(m_lfSteerWattHoursPath, value, m_lfSteerWattHoursUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_DRIVE_POWER:
            SignalLogger::WriteDouble(m_lfDrivePowerPath, value, m_lfDrivePowerUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_DRIVE_CURRENT:
            SignalLogger::WriteDouble(m_lfDriveCurrentPath, value, m_lfDriveCurrentUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_DRIVE_TOTALPOWER:
            SignalLogger::WriteDouble(m_lfDriveTotalPowerPath, value, m_lfDriveTotalPowerUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_DRIVE_WATT_HOURS:
            SignalLogger::WriteDouble(m_lfDriveWattHoursPath, value, m_lfDriveWattHoursUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_STEER_POWER:
            SignalLogger::WriteDouble(m_rfSteerPowerPath, value, m_rfSteerPowerUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_STEER_CURRENT:
            SignalLogger::WriteDouble(m_rfSteerCurrentPath, value, m_rfSteerCurrentUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_STEER_TOTALPOWER:
            SignalLogger::WriteDouble(m_rfSteerTotalPowerPath, value, m_rfSteerTotalPowerUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_STEER_WATT_HOURS:
            SignalLogger::WriteDouble(m_rfSteerWattHoursPath, value, m_rfSteerWattHoursUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_DRIVE_POWER:
            SignalLogger::WriteDouble(m_rfDrivePowerPath, value, m_rfDrivePowerUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_DRIVE_CURRENT:
            SignalLogger::WriteDouble(m_rfDriveCurrentPath, value, m_rfDriveCurrentUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_DRIVE_TOTALPOWER:
            SignalLogger::WriteDouble(m_rfDriveTotalPowerPath, value, m_rfDriveTotalPowerUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_DRIVE_WATT_HOURS:
            SignalLogger::WriteDouble(m_rfDriveWattHoursPath, value, m_rfDriveWattHoursUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_STEER_POWER:
            SignalLogger::WriteDouble(m_lbSteerPowerPath, value, m_lbSteerPowerUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_STEER_CURRENT:
            SignalLogger::WriteDouble(m_lbSteerCurrentPath, value, m_lbSteerCurrentUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_STEER_TOTALPOWER:
            SignalLogger::WriteDouble(m_lbSteerTotalPowerPath, value, m_lbSteerTotalPowerUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_STEER_WATT_HOURS:
            SignalLogger::WriteDouble(m_lbSteerWattHoursPath, value, m_lbSteerWattHoursUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_DRIVE_POWER:
            SignalLogger::WriteDouble(m_lbDrivePowerPath, value, m_lbDrivePowerUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_DRIVE_CURRENT:
            SignalLogger::WriteDouble(m_lbDriveCurrentPath, value, m_lbDriveCurrentUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_DRIVE_TOTALPOWER:
            SignalLogger::WriteDouble(m_lbDriveTotalPowerPath, value, m_lbDriveTotalPowerUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_DRIVE_WATT_HOURS:
            SignalLogger::WriteDouble(m_lbDriveWattHoursPath, value, m_lbDriveWattHoursUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_STEER_POWER:
            SignalLogger::WriteDouble(m_rbSteerPowerPath, value, m_rbSteerPowerUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_STEER_CURRENT:
            SignalLogger::WriteDouble(m_rbSteerCurrentPath, value, m_rbSteerPowerUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_STEER_TOTALPOWER:
            SignalLogger::WriteDouble(m_rbSteerTotalPowerPath, value, m_rbSteerTotalPowerUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_STEER_WATT_HOURS:
            SignalLogger::WriteDouble(m_rbSteerWattHoursPath, value, m_rbSteerWattHoursUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_DRIVE_POWER:
            SignalLogger::WriteDouble(m_rbDrivePowerPath, value, m_rbDrivePowerUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_DRIVE_CURRENT:
            SignalLogger::WriteDouble(m_rbDriveCurrentPath, value, m_rbDriveCurrentUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_DRIVE_TOTALPOWER:
            SignalLogger::WriteDouble(m_rbDriveTotalPowerPath, value, m_rbDriveTotalPowerUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_DRIVE_WATT_HOURS:
            SignalLogger::WriteDouble(m_rbDriveWattHoursPath, value, m_rbDriveWattHoursUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::SWERVE_CHASSIS_TOTAL_CURRENT:
            SignalLogger::WriteDouble(m_swerveChassisTotalCurrentPath, value, m_swerveChassisTotalCurrentUnits, m_latency);
            break;

        case DragonDataLogger::DoubleSignals::SWERVE_CHASSIS_WATT_HOURS:
            SignalLogger::WriteDouble(m_swerveChassisWattHoursPath, value, m_swerveChassisWattHoursUnits, m_latency);
            break;
        case DragonDataLogger::DoubleSignals::LIMELIGHT1_NUMBER_OF_TAGS:
            SignalLogger::WriteDouble(m_limelight1NumberOfTagsPath, value, m_limelight1NumberOfTagsUnits, m_latency);
            break;
        case DragonDataLogger::DoubleSignals::LIMELIGHT1_NUMBER_OF_ALGAE:
            SignalLogger::WriteDouble(m_limelight1NumberOfAlgaePath, value, m_limelight1NumberOfAlgaeUnits, m_latency);
            break;
        default:
            break;
        }
    }
}

void DragonDataLogger::LogStringData(uint64_t timestamp, DragonDataLogger::StringSignals signalID, std::string value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr != nullptr)
    {
        switch (signalID)
        {
        case DragonDataLogger::StringSignals::CHASSIS_DRIVE_STATE:
            SignalLogger::WriteString(m_driveStatePath, value, m_latency);
            break;

        case DragonDataLogger::StringSignals::CHASSIS_HEADING_STATE:
            SignalLogger::WriteString(m_headingStatePath, value, m_latency);
            break;

        default:
            break;
        }
    }
}
void DragonDataLogger::Log2DPoseData(uint64_t timestamp, DragonDataLogger::PoseSingals signalID, frc::Pose2d value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr != nullptr)
    {
        switch (signalID)
        {
        case DragonDataLogger::PoseSingals::CURRENT_CHASSIS_POSE2D:
        {
            double x = value.X().value();
            double y = value.Y().value();
            double rot = value.Rotation().Radians().value();
            std::vector<double> pose = {x, y, rot};
            SignalLogger::WriteDoubleArray(m_chassisPose2dPath, pose, m_pose2dUnits, m_latency);
            break;
        }
        case DragonDataLogger::PoseSingals::VISION_DRIVE_TO_LEFT_REEF_BRANCH_TARGET_POSE:
        {
            double x = value.X().value();
            double y = value.Y().value();
            double rot = value.Rotation().Radians().value();
            std::vector<double> pose = {x, y, rot};
            SignalLogger::WriteDoubleArray(m_visionDriveLBranchPose2dPath, pose, m_visionDriveLBranchPose2dPath, m_latency);
            break;
        }
        case DragonDataLogger::PoseSingals::VISION_DRIVE_TO_RIGHT_REEF_BRANCH_TARGET_POSE:
        {
            double x = value.X().value();
            double y = value.Y().value();
            double rot = value.Rotation().Radians().value();
            std::vector<double> pose = {x, y, rot};
            SignalLogger::WriteDoubleArray(m_visionDriveRBranchPose2dPath, pose, m_visionDriveRBranchPose2dPath, m_latency);
            break;
        }
        case DragonDataLogger::PoseSingals::VISION_DRIVE_TO_CORAL_STATION_TARGET_POSE:
        {
            double x = value.X().value();
            double y = value.Y().value();
            double rot = value.Rotation().Radians().value();
            std::vector<double> pose = {x, y, rot};
            SignalLogger::WriteDoubleArray(m_visionDriveCoralStationPose2dPath, pose, m_visionDriveCoralStationPose2dUnits, m_latency);
            break;
            ;
        }
        case DragonDataLogger::PoseSingals::ODOMETRY_DRIVE_TO_LEFT_REEF_BRANCH_TARGET_POSE:
        {
            double x = value.X().value();
            double y = value.Y().value();
            double rot = value.Rotation().Radians().value();
            std::vector<double> pose = {x, y, rot};
            SignalLogger::WriteDoubleArray(m_odometryDriveLBranchPose2dPath, pose, m_odometryDriveLBranchPose2dUnits, m_latency);
            break;
        }
        case DragonDataLogger::PoseSingals::ODOMETRY_DRIVE_TO_RIGHT_REEF_BRANCH_TARGET_POSE:
        {
            double x = value.X().value();
            double y = value.Y().value();
            double rot = value.Rotation().Radians().value();
            std::vector<double> pose = {x, y, rot};
            SignalLogger::WriteDoubleArray(m_odometryDriveRBranchPose2dPath, pose, m_odometryDriveRBranchPose2dUnits, m_latency);
            break;
        }
        case DragonDataLogger::PoseSingals::ODOMETRY_DRIVE_TO_CORAL_STATION_TARGET_POSE:
        {
            double x = value.X().value();
            double y = value.Y().value();
            double rot = value.Rotation().Radians().value();
            std::vector<double> pose = {x, y, rot};
            SignalLogger::WriteDoubleArray(m_odometryDriveCoralStationPose2dPath, pose, m_odometryDriveCoralStationPose2dUnits, m_latency);
            break;
        }
        case DragonDataLogger::PoseSingals::CURRENT_CHASSIS_QUEST_POSE2D:
        {
            double x = value.X().value();
            double y = value.Y().value();
            double rot = value.Rotation().Radians().value();
            std::vector<double> pose = {x, y, rot};
            SignalLogger::WriteDoubleArray(m_questPose2dPath, pose, m_questPose2dUnits, m_latency);
            break;
        }
        default:
            break;
        }
    }
}

void DragonDataLogger::Log3DPoseData(uint64_t timestamp, DragonDataLogger::PoseSingals signalID, frc::Pose3d value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr != nullptr)
    {
        switch (signalID)
        {
        case DragonDataLogger::PoseSingals::CURRENT_CHASSIS_LIMELIGHT_POSE3D:
        {
            double x = value.ToPose2d().X().value();
            double y = value.ToPose2d().Y().value();
            double rot = value.ToPose2d().Rotation().Radians().value();
            std::vector<double> pose = {x, y, rot};
            SignalLogger::WriteDoubleArray(m_limelight1Pose3dPath, pose, m_limelight2Pose3dPath, m_latency);
            break;
        }
        case DragonDataLogger::PoseSingals::CURRENT_CHASSIS_LIMELIGHT2_POSE3D:
        {
            double x = value.ToPose2d().X().value();
            double y = value.ToPose2d().Y().value();
            double rot = value.ToPose2d().Rotation().Radians().value();
            std::vector<double> pose = {x, y, rot};
            SignalLogger::WriteDoubleArray(m_limelight2Pose3dPath, pose, m_limelight2Pose3dPath, m_latency);
            break;
        }
        default:
            break;
        }
    }
}

void DragonDataLogger::LogSwerveModuleStateData(uint64_t timestamp, DragonDataLogger::SwerveStateSingals signalID, frc::SwerveModuleState value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr != nullptr)
    {
        switch (signalID)
        {
        case DragonDataLogger::SwerveStateSingals::TARGET_LEFT_FRONT_STATE:
        {
            double speed = value.speed.value();
            double angle = value.angle.Radians().value();
            SignalLogger::WriteDouble(m_frontLeftTargetSpeedPath, speed, m_swerveModuleStateUnits, m_latency);
            SignalLogger::WriteDouble(m_frontLeftTargetAnglePath, angle, m_swerveModuleStateUnits, m_latency);
            break;
        }

        case DragonDataLogger::SwerveStateSingals::TARGET_LEFT_BACK_STATE:
        {
            double speed = value.speed.value();
            double angle = value.angle.Radians().value();
            SignalLogger::WriteDouble(m_backLeftTargetSpeedPath, speed, m_swerveModuleStateUnits, m_latency);
            SignalLogger::WriteDouble(m_backLeftTargetAnglePath, angle, m_swerveModuleStateUnits, m_latency);
            break;
        }

        case DragonDataLogger::SwerveStateSingals::TARGET_RIGHT_FRONT_STATE:
        {
            double speed = value.speed.value();
            double angle = value.angle.Radians().value();
            SignalLogger::WriteDouble(m_frontRightTargetSpeedPath, speed, m_swerveModuleStateUnits, m_latency);
            SignalLogger::WriteDouble(m_frontRightTargetAnglePath, angle, m_swerveModuleStateUnits, m_latency);
            break;
        }

        case DragonDataLogger::SwerveStateSingals::TARGET_RIGHT_BACK_STATE:
        {
            double speed = value.speed.value();
            double angle = value.angle.Radians().value();
            SignalLogger::WriteDouble(m_backRightTargetSpeedPath, speed, m_swerveModuleStateUnits, m_latency);
            SignalLogger::WriteDouble(m_backRightTargetAnglePath, angle, m_swerveModuleStateUnits, m_latency);
            break;
        }

        case DragonDataLogger::SwerveStateSingals::ACTUAL_LEFT_FRONT_STATE:
        {
            double speed = value.speed.value();
            double angle = value.angle.Radians().value();
            SignalLogger::WriteDouble(m_frontLeftActualSpeedPath, speed, m_swerveModuleStateUnits, m_latency);
            SignalLogger::WriteDouble(m_frontLeftActualAnglePath, angle, m_swerveModuleStateUnits, m_latency);
            break;
        }

        case DragonDataLogger::SwerveStateSingals::ACTUAL_LEFT_BACK_STATE:
        {
            double speed = value.speed.value();
            double angle = value.angle.Radians().value();
            SignalLogger::WriteDouble(m_backLeftActualSpeedPath, speed, m_swerveModuleStateUnits, m_latency);
            SignalLogger::WriteDouble(m_backLeftActualAnglePath, angle, m_swerveModuleStateUnits, m_latency);
            break;
        }

        case DragonDataLogger::SwerveStateSingals::ACTUAL_RIGHT_FRONT_STATE:
        {
            double speed = value.speed.value();
            double angle = value.angle.Radians().value();
            SignalLogger::WriteDouble(m_frontRightActualSpeedPath, speed, m_swerveModuleStateUnits, m_latency);
            SignalLogger::WriteDouble(m_frontRightActualAnglePath, angle, m_swerveModuleStateUnits, m_latency);
            break;
        }

        case DragonDataLogger::SwerveStateSingals::ACTUAL_RIGHT_BACK_STATE:
        {
            double speed = value.speed.value();
            double angle = value.angle.Radians().value();
            SignalLogger::WriteDouble(m_backRightActualSpeedPath, speed, m_swerveModuleStateUnits, m_latency);
            SignalLogger::WriteDouble(m_backRightActualAnglePath, angle, m_swerveModuleStateUnits, m_latency);
            break;
        }

        default:
            break;
        }
    }
}

void DragonDataLogger::LogChassisSpeedsData(uint64_t timestamp, DragonDataLogger::ChassisSpeedSignals signalID, frc::ChassisSpeeds value)
{
    auto dataMgr = DragonDataLoggerMgr::GetInstance();
    if (dataMgr != nullptr)
    {
        // TODO:  need to compare/store; need to do element by element
        switch (signalID)
        {
        case DragonDataLogger::ChassisSpeedSignals::ACTUAL_SPEEDS:
        {
            double vx = value.vx.value();
            double vy = value.vy.value();
            double omega = value.omega.value();
            SignalLogger::WriteDouble(m_swerveActualvxPath, vx, m_swerveChassisSpeedUnits, m_latency);
            SignalLogger::WriteDouble(m_swerveActualvyPath, vy, m_swerveChassisSpeedUnits, m_latency);
            SignalLogger::WriteDouble(m_swerveActualOmegaPath, omega, m_swerveChassisSpeedUnits, m_latency);
            break;
        }
        case DragonDataLogger::ChassisSpeedSignals::TARGET_SPEEDS:
        {
            double vx = value.vx.value();
            double vy = value.vy.value();
            double omega = value.omega.value();
            SignalLogger::WriteDouble(m_swerveTargetvxPath, vx, m_swerveChassisSpeedUnits, m_latency);
            SignalLogger::WriteDouble(m_swerveTargetvyPath, vy, m_swerveChassisSpeedUnits, m_latency);
            SignalLogger::WriteDouble(m_swerveTargetOmegaPath, omega, m_swerveChassisSpeedUnits, m_latency);
            break;
        }
        default:
            break;
        }
    }
}
