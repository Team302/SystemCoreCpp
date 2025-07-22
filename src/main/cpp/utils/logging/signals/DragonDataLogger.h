
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
#include <string>

#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Pose3d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "units/time.h"
#include "utils/logging/signals/DragonDataLoggerSignals.h"

using namespace std;

class DragonDataLogger
{
public:
    DragonDataLogger();
    virtual ~DragonDataLogger() = default;

    virtual void DataLog(uint64_t timestamp) = 0;

    // when adding a signal:
    // - Add to the appropriate enum
    // - Add a signal variable below
    // - Construct the signal in DragonDataLoggerSignals::DragonDataLoggerSignals()
    // - Add Case statement in the appropriate helper method in DragonDataLogger

    enum BoolSignals
    {
        IS_BROWNOUT,
        DRIVE_TO_IS_DONE,
        IS_ALGAE_DETECTED
    };

    enum DoubleSignals
    {
        CHASSIS_STORED_HEADING_DEGREES,
        CHASSIS_YAW_DEGREES,
        ELECTRICAL_VOLTAGE,
        ELECTRICAL_CURRENT,
        ELECTRICAL_POWER,
        LIMELIGHT_TV_1,
        LIMELIGHT_TX_1,
        LIMELIGHT_TY_1,
        LIMELIGHT_FIDUCIAL_ID_1,
        BATTERY_VOLTAGE,
        BROWNOUT_VOLTAGE,
        INPUT_VOLTAGE,
        INPUT_CURRENT,
        CPU_TEMP,
        LEFT_FRONT_SWERVE_STEER_POWER,
        LEFT_FRONT_SWERVE_STEER_CURRENT,
        LEFT_FRONT_SWERVE_STEER_TOTALPOWER,
        LEFT_FRONT_SWERVE_STEER_WATT_HOURS,
        LEFT_FRONT_SWERVE_DRIVE_POWER,
        LEFT_FRONT_SWERVE_DRIVE_CURRENT,
        LEFT_FRONT_SWERVE_DRIVE_TOTALPOWER,
        LEFT_FRONT_SWERVE_DRIVE_WATT_HOURS,
        RIGHT_FRONT_SWERVE_STEER_POWER,
        RIGHT_FRONT_SWERVE_STEER_CURRENT,
        RIGHT_FRONT_SWERVE_STEER_TOTALPOWER,
        RIGHT_FRONT_SWERVE_STEER_WATT_HOURS,
        RIGHT_FRONT_SWERVE_DRIVE_POWER,
        RIGHT_FRONT_SWERVE_DRIVE_CURRENT,
        RIGHT_FRONT_SWERVE_DRIVE_TOTALPOWER,
        RIGHT_FRONT_SWERVE_DRIVE_WATT_HOURS,
        LEFT_BACK_SWERVE_STEER_POWER,
        LEFT_BACK_SWERVE_STEER_CURRENT,
        LEFT_BACK_SWERVE_STEER_TOTALPOWER,
        LEFT_BACK_SWERVE_STEER_WATT_HOURS,
        LEFT_BACK_SWERVE_DRIVE_POWER,
        LEFT_BACK_SWERVE_DRIVE_CURRENT,
        LEFT_BACK_SWERVE_DRIVE_TOTALPOWER,
        LEFT_BACK_SWERVE_DRIVE_WATT_HOURS,
        RIGHT_BACK_SWERVE_STEER_POWER,
        RIGHT_BACK_SWERVE_STEER_CURRENT,
        RIGHT_BACK_SWERVE_STEER_TOTALPOWER,
        RIGHT_BACK_SWERVE_STEER_WATT_HOURS,
        RIGHT_BACK_SWERVE_DRIVE_POWER,
        RIGHT_BACK_SWERVE_DRIVE_CURRENT,
        RIGHT_BACK_SWERVE_DRIVE_TOTALPOWER,
        RIGHT_BACK_SWERVE_DRIVE_WATT_HOURS,
        SWERVE_CHASSIS_TOTAL_CURRENT,
        SWERVE_CHASSIS_WATT_HOURS,
        LIMELIGHT1_NUMBER_OF_TAGS,
        LIMELIGHT1_NUMBER_OF_ALGAE
    };

    enum StringSignals
    {
        CHASSIS_HEADING_STATE,
        CHASSIS_DRIVE_STATE,
        AUTON_PATH_NAME
    };

    enum PoseSingals
    {
        CURRENT_CHASSIS_POSE2D,
        CURRENT_CHASSIS_LIMELIGHT_POSE3D,
        CURRENT_CHASSIS_LIMELIGHT2_POSE3D,
        CURRENT_CHASSIS_QUEST_POSE2D,
        VISION_DRIVE_TO_LEFT_REEF_BRANCH_TARGET_POSE,
        VISION_DRIVE_TO_RIGHT_REEF_BRANCH_TARGET_POSE,
        VISION_DRIVE_TO_CORAL_STATION_TARGET_POSE,
        ODOMETRY_DRIVE_TO_LEFT_REEF_BRANCH_TARGET_POSE,
        ODOMETRY_DRIVE_TO_RIGHT_REEF_BRANCH_TARGET_POSE,
        ODOMETRY_DRIVE_TO_CORAL_STATION_TARGET_POSE

    };

    enum ChassisSpeedSignals
    {
        TARGET_SPEEDS,
        ACTUAL_SPEEDS
    };

    enum SwerveStateSingals
    {
        TARGET_LEFT_FRONT_STATE,
        TARGET_RIGHT_FRONT_STATE,
        TARGET_LEFT_BACK_STATE,
        TARGET_RIGHT_BACK_STATE,
        ACTUAL_LEFT_FRONT_STATE,
        ACTUAL_RIGHT_FRONT_STATE,
        ACTUAL_LEFT_BACK_STATE,
        ACTUAL_RIGHT_BACK_STATE
    };

    // initialize these signals in the constructor

    string m_brownOutPath = "/RoboRio/IsBrownOut";

    string m_storedHeadingPath = "/Chassis/StoredHeading";
    string m_storedHeadingUnits = "Degrees";

    string m_ChassisYawPath = "/Chassis/Yaw";
    string m_ChassisYawUnits = "Degrees";

    string m_electricalVoltagePath = "/Electrical/Voltage";
    string m_electricalVoltageUnits = "volts";
    string m_electricalCurrentPath = "/Electrical/Current";
    string m_electricalCurrentUnits = "";
    string m_electricalPowerPath = "";
    string m_electricalPowerUnits = "";

    string m_txPath = "LL1/tx";
    string m_txUnits = "";
    string m_tyPath = "LL1/ty";
    string m_tyUnits = "";
    string m_tvPath = "LL1/tv";
    string m_tvUnits = "";
    string m_fiducialPath = "LL1/fiducial";
    string m_fiducialUnits = "";

    // RIO

    string m_batteryVoltagePath = "/RoboRio/BatteryVoltage";
    string m_batteryVoltageUnits = "Volts";
    string m_brownoutVoltagePath = "/RoboRio/BrownoutVoltage";
    string m_brownoutVoltageUnits = "Volts";
    string m_inputVoltagePath = "/RoboRio/InputVoltage";
    string m_inputVoltageUnits = "Volts";
    string m_inputCurrentPath = "/RoboRio/InputCurrent";
    string m_inputCurrentUnits = "Amps";
    string m_cpuTempPath = "/RoboRio/CPUTemp";
    string m_cpuTempUnits = "Degrees C";

    string m_lfSteerPowerPath = "/Chassis/FrontLeftModule/Steer/Power";
    string m_lfSteerPowerUnits = "";
    string m_lfSteerCurrentPath = "/Chassis/FrontLeftModule/Steer/Current";
    string m_lfSteerCurrentUnits = "Amps";
    string m_lfSteerTotalPowerPath = "/Chassis/FrontLeftModule/Steer/TotalPower";
    string m_lfSteerTotalPowerUnits = "";
    string m_lfSteerWattHoursPath = "/Chassis/FrontLeftModule/Steer/WattHours";
    string m_lfSteerWattHoursUnits = "WattHours";

    string m_lfDrivePowerPath = "/Chassis/FrontLeftModule/Drive/Power";
    string m_lfDrivePowerUnits = "";
    string m_lfDriveCurrentPath = "/Chassis/FrontLeftModule/Drive/Current";
    string m_lfDriveCurrentUnits = "Amps";
    string m_lfDriveTotalPowerPath = "/Chassis/FrontLeftModule/Drive/TotalPower";
    string m_lfDriveTotalPowerUnits = "";
    string m_lfDriveWattHoursPath = "/Chassis/FrontLeftModule/Drive/WattHours";
    string m_lfDriveWattHoursUnits = "WattHours";

    string m_rfSteerPowerPath = "/Chassis/FrontRightModule/Steer/Power";
    string m_rfSteerPowerUnits = "";
    string m_rfSteerCurrentPath = "/Chassis/FrontRightModule/Steer/Current";
    string m_rfSteerCurrentUnits = "Amps";
    string m_rfSteerTotalPowerPath = "/Chassis/FrontRightModule/Steer/TotalPower";
    string m_rfSteerTotalPowerUnits = "";
    string m_rfSteerWattHoursPath = "/Chassis/FrontRightModule/Steer/WattHours";
    string m_rfSteerWattHoursUnits = "WattHours";

    string m_rfDrivePowerPath = "/Chassis/FrontRightModule/Drive/Power";
    string m_rfDrivePowerUnits = "";
    string m_rfDriveCurrentPath = "/Chassis/FrontRightModule/Drive/Current";
    string m_rfDriveCurrentUnits = "Amps";
    string m_rfDriveTotalPowerPath = "/Chassis/FrontRightModule/Drive/TotalPower";
    string m_rfDriveTotalPowerUnits = "";
    string m_rfDriveWattHoursPath = "/Chassis/FrontRightModule/Drive/WattHours";
    string m_rfDriveWattHoursUnits = "WattHours";

    string m_lbSteerPowerPath = "/Chassis/BackLeftModule/Steer/Power";
    string m_lbSteerPowerUnits = "";
    string m_lbSteerCurrentPath = "/Chassis/BackLeftModule/Steer/Current";
    string m_lbSteerCurrentUnits = "Amps";
    string m_lbSteerTotalPowerPath = "/Chassis/BackLeftModule/Steer/TotalPower";
    string m_lbSteerTotalPowerUnits = "";
    string m_lbSteerWattHoursPath = "/Chassis/BackLeftModule/Steer/WattHours";
    string m_lbSteerWattHoursUnits = "WattHours";

    string m_lbDrivePowerPath = "/Chassis/BackLeftModule/Drive/Power";
    string m_lbDrivePowerUnits = "";
    string m_lbDriveCurrentPath = "/Chassis/BackLeftModule/Drive/Current";
    string m_lbDriveCurrentUnits = "Amps";
    string m_lbDriveTotalPowerPath = "/Chassis/BackLeftModule/Drive/TotalPower";
    string m_lbDriveTotalPowerUnits = "";
    string m_lbDriveWattHoursPath = "/Chassis/BackLeftModule/Drive/WattHours";
    string m_lbDriveWattHoursUnits = "WattHours";

    string m_rbSteerPowerPath = "/Chassis/BackRightModule/Steer/Power";
    string m_rbSteerPowerUnits = "";
    string m_rbSteerCurrentPath = "/Chassis/BackRightModule/Steer/Current";
    string m_rbSteerCurrentUnits = "Amps";
    string m_rbSteerTotalPowerPath = "/Chassis/BackRightModule/Steer/TotalPower";
    string m_rbSteerTotalPowerUnits = "";
    string m_rbSteerWattHoursPath = "/Chassis/BackRightModule/Steer/WattHours";
    string m_rbSteerWattHoursUnits = "WattHours";

    string m_rbDrivePowerPath = "/Chassis/BackRightModule/Drive/Power";
    string m_rbDrivePowerUnits = "";
    string m_rbDriveCurrentPath = "/Chassis/BackRightModule/Drive/Current";
    string m_rbDriveCurrentUnits = "Amps";
    string m_rbDriveTotalPowerPath = "/Chassis/BackRightModule/Drive/TotalPower";
    string m_rbDriveTotalPowerUnits = "";
    string m_rbDriveWattHoursPath = "/Chassis/BackRightModule/Drive/WattHours";
    string m_rbDriveWattHoursUnits = "WattHours";

    string m_swerveChassisTotalPowerPath = "/Chassis/TotalPower";
    string m_swerveChassisTotalPowerUnits = "";
    string m_swerveChassisWattHoursPath = "/Chassis/WattHours";
    string m_swerveChassisWattHoursUnits = "";
    string m_swerveChassisTotalCurrentPath = "/Chassis/TotalCurrent";
    string m_swerveChassisTotalCurrentUnits = "";
    string m_limelight1NumberOfTagsPath = "LL1/NumberOfTags";
    string m_limelight1NumberOfTagsUnits = "";
    string m_limelight1NumberOfAlgaePath = "LL1/NumberOfAlgae";
    string m_limelight1NumberOfAlgaeUnits = "";
    string m_headingStatePath = "/Chassis/HeadingState";
    string m_driveStatePath = "/Chassis/DriveState";
    string m_IsDonePath = "/Chassis/IsDone";
    string m_IsAlgaeDetected = "/Chassis/IsAlgaeDetected";

    string m_chassisPose2dPath = "/Chassis/Pose2d";
    string m_visionDriveLBranchPose2dPath = "/Vision/DriveToLeftReefBranchPose2d";
    string m_visionDriveLBranchPose2dUnits = "X, Y, Rotation";
    string m_visionDriveRBranchPose2dPath = "/Vision/DriveToRightReefBranchPose2d";
    string m_visionDriveRBranchPose2dUnits = "X, Y, Rotation";
    string m_visionDriveCoralStationPose2dPath = "/Vision/DriveToCoralStationPose2d";
    string m_visionDriveCoralStationPose2dUnits = "X, Y, Rotation";
    string m_odometryDriveLBranchPose2dPath = "/Odometry/DriveToLeftReefBranchPose2d";
    string m_odometryDriveLBranchPose2dUnits = "X, Y, Rotation";
    string m_odometryDriveRBranchPose2dPath = "/Odometry/DriveToRightReefBranchPose2d";
    string m_odometryDriveRBranchPose2dUnits = "X, Y, Rotation";
    string m_odometryDriveCoralStationPose2dPath = "/Odometry/DriveToCoralStationPose2d";
    string m_odometryDriveCoralStationPose2dUnits = "X, Y, Rotation";
    string m_questPose2dPath = "/Chassis/QuestPose2d";
    string m_questPose2dUnits = "X, Y, Rotation";

    string m_limelight1Pose3dPath = "LL1/Pose3d";
    string m_limelight1Pose3dUnits = "LL1/ X, Y, Rotation";
    string m_limelight2Pose3dPath = "LL2/Pose3d";
    string m_limelight2Pose3dUnits = "LL2/ X, Y, Rotation";
    string m_pose2dUnits = "X, Y, Rotation";

    string m_frontLeftTargetSpeedPath = "/Chassis/FrontLeftModule/TargetState/Speed";
    string m_frontLeftTargetAnglePath = "/Chassis/FrontLeftModule/TargetState/Angle";
    string m_frontRightTargetSpeedPath = "/Chassis/FrontRightModule/TargetState/Speed";
    string m_frontRightTargetAnglePath = "/Chassis/FrontRightModule/TargetState/Angle";
    string m_backLeftTargetSpeedPath = "/Chassis/BackLeftModule/TargetState/Speed";
    string m_backLeftTargetAnglePath = "/Chassis/BackLeftModule/TargetState/Angle";
    string m_backRightTargetSpeedPath = "/Chassis/BackRightModule/TargetState/Speed";
    string m_backRightTargetAnglePath = "/Chassis/BackRightModule/TargetState/Angle";
    string m_frontLeftActualSpeedPath = "/Chassis/FrontLeftModule/ActualState/Speed";
    string m_frontLeftActualAnglePath = "/Chassis/FrontLeftModule/ActualState/Angle";
    string m_frontRightActualSpeedPath = "/Chassis/FrontRightModule/ActualState/Speed";
    string m_frontRightActualAnglePath = "/Chassis/FrontRightModule/ActualState/Angle";
    string m_backLeftActualSpeedPath = "/Chassis/BackLeftModule/ActualState/Speed";
    string m_backLeftActualAnglePath = "/Chassis/BackLeftModule/ActualState/Angle";
    string m_backRightActualSpeedPath = "/Chassis/BackRightModule/ActualState/Speed";
    string m_backRightActualAnglePath = "/Chassis/BackRightModule/ActualState/Angle";

    string m_swerveTargetvxPath = "/Chassis/TargetSpeeds/Vx";
    string m_swerveTargetvyPath = "/Chassis/TargetSpeeds/Vy";
    string m_swerveTargetOmegaPath = "/Chassis/TargetSpeeds/Omega";

    string m_swerveActualvxPath = "/Chassis/ActualSpeeds/Vx";
    string m_swerveActualvyPath = "/Chassis/ActualSpeeds/Vy";
    string m_swerveActualOmegaPath = "/Chassis/ActualSpeeds/Omega";

    string m_swerveModuleStateUnits = "Speed, Angle";
    string m_swerveChassisSpeedUnits = "Vx, Vy, Omega";

    units::time::second_t m_latency = units::time::second_t(0);

protected:
    void LogBoolData(uint64_t timestamp, DragonDataLogger::BoolSignals signalID, bool value);
    void LogDoubleData(uint64_t timestamp, DragonDataLogger::DoubleSignals signalID, double value);
    void LogStringData(uint64_t timestamp, DragonDataLogger::StringSignals signalID, string value);
    void Log2DPoseData(uint64_t timestamp, DragonDataLogger::PoseSingals signalID, frc::Pose2d value);
    void Log3DPoseData(uint64_t timestamp, DragonDataLogger::PoseSingals signalID, frc::Pose3d value);

    void LogSwerveModuleStateData(uint64_t timestamp, DragonDataLogger::SwerveStateSingals signalID, frc::SwerveModuleState value);
    void LogChassisSpeedsData(uint64_t timestamp, DragonDataLogger::ChassisSpeedSignals signalID, frc::ChassisSpeeds value);

    const double m_doubleTolerance = 0.001;
};
