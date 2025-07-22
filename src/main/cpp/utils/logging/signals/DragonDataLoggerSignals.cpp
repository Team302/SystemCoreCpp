
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

#include "frc/DataLogManager.h"
#include "utils/logging/signals/DragonDataLoggerSignals.h"

DragonDataLoggerSignals *DragonDataLoggerSignals::m_instance = nullptr;
DragonDataLoggerSignals *DragonDataLoggerSignals::GetInstance()
{
    if (DragonDataLoggerSignals::m_instance == nullptr)
    {
        DragonDataLoggerSignals::m_instance = new DragonDataLoggerSignals();
    }
    return DragonDataLoggerSignals::m_instance;
}

DragonDataLoggerSignals::DragonDataLoggerSignals()
{
    // wpi::log::DataLog &log = frc::DataLogManager::GetLog();

    // m_isBrownOut = wpi::log::BooleanLogEntry(log, "/RoboRio/IsBrownOut");
    // m_batteryVoltage = wpi::log::DoubleLogEntry(log, "/RoboRio/BatteryVoltage");
    // m_brownoutVoltage = wpi::log::DoubleLogEntry(log, "/RoboRio/BrownoutVoltage");
    // m_inputVoltage = wpi::log::DoubleLogEntry(log, "/RoboRio/InputVoltage");
    // m_inputCurrent = wpi::log::DoubleLogEntry(log, "/RoboRio/InputCurrent");
    // m_cpuTemp = wpi::log::DoubleLogEntry(log, "/RoboRio/CPUTemp");

    // m_storedHeading = wpi::log::DoubleLogEntry(log, "/Chassis/StoredHeading(Degrees)");
    // m_storedHeading.Append(0.0);
    // m_chassisYaw = wpi::log::DoubleLogEntry(log, "/Chassis/Yaw(Degrees)");
    // m_chassisYaw.Append(0.0);

    // m_lfSteerPower = wpi::log::DoubleLogEntry(log, "/Chassis/FrontLeftModule/Steer/Power");
    // m_lfSteerEnergy = wpi::log::DoubleLogEntry(log, "/Chassis/FrontLeftModule/Steer/Energy");
    // m_lfSteerTotalPower = wpi::log::DoubleLogEntry(log, "/Chassis/FrontLeftModule/Steer/TotalPower");
    // m_lfSteerWattHours = wpi::log::DoubleLogEntry(log, "/Chassis/FrontLeftModule/Steer/WattHours");

    // m_lfDrivePower = wpi::log::DoubleLogEntry(log, "/Chassis/FrontLeftModule/Drive/Power");
    // m_lfDriveEnergy = wpi::log::DoubleLogEntry(log, "/Chassis/FrontLeftModule/Drive/Energy");
    // m_lfDriveTotalPower = wpi::log::DoubleLogEntry(log, "/Chassis/FrontLeftModule/Drive/TotalPower");
    // m_lfDriveWattHours = wpi::log::DoubleLogEntry(log, "/Chassis/FrontLeftModule/Drive/WattHours");

    // m_rfSteerPower = wpi::log::DoubleLogEntry(log, "/Chassis/FrontRightModule/Steer/Power");
    // m_rfSteerEnergy = wpi::log::DoubleLogEntry(log, "/Chassis/FrontRightModule/Steer/Energy");
    // m_rfSteerTotalPower = wpi::log::DoubleLogEntry(log, "/Chassis/FrontRightModule/Steer/TotalPower");
    // m_rfSteerWattHours = wpi::log::DoubleLogEntry(log, "/Chassis/FrontRightModule/Steer/WattHours");

    // m_rfDrivePower = wpi::log::DoubleLogEntry(log, "/Chassis/FrontRightModule/Drive/Power");
    // m_rfDriveEnergy = wpi::log::DoubleLogEntry(log, "/Chassis/FrontRightModule/Drive/Energy");
    // m_rfDriveTotalPower = wpi::log::DoubleLogEntry(log, "/Chassis/FrontRightModule/Drive/TotalPower");
    // m_rfDriveWattHours = wpi::log::DoubleLogEntry(log, "/Chassis/FrontRightModule/Drive/WattHours");

    // m_lbSteerPower = wpi::log::DoubleLogEntry(log, "/Chassis/BackLeftModule/Steer/Power");
    // m_lbSteerEnergy = wpi::log::DoubleLogEntry(log, "/Chassis/BackLeftModule/Steer/Energy");
    // m_lbSteerTotalPower = wpi::log::DoubleLogEntry(log, "/Chassis/BackLeftModule/Steer/TotalPower");
    // m_lbSteerWattHours = wpi::log::DoubleLogEntry(log, "/Chassis/BackLeftModule/Steer/WattHours");

    // m_lbDrivePower = wpi::log::DoubleLogEntry(log, "/Chassis/BackLeftModule/Drive/Power");
    // m_lbDriveEnergy = wpi::log::DoubleLogEntry(log, "/Chassis/BackLeftModule/Drive/Energy");
    // m_lbDriveTotalPower = wpi::log::DoubleLogEntry(log, "/Chassis/BackLeftModule/Drive/TotalPower");
    // m_lbDriveWattHours = wpi::log::DoubleLogEntry(log, "/Chassis/BackLeftModule/Drive/WattHours");

    // m_rbSteerPower = wpi::log::DoubleLogEntry(log, "/Chassis/BackRightModule/Steer/Power");
    // m_rbSteerEnergy = wpi::log::DoubleLogEntry(log, "/Chassis/BackRightModule/Steer/Energy");
    // m_rbSteerTotalPower = wpi::log::DoubleLogEntry(log, "/Chassis/BackRightModule/Steer/TotalPower");
    // m_rbSteerWattHours = wpi::log::DoubleLogEntry(log, "/Chassis/BackRightModule/Steer/WattHours");

    // m_rbDrivePower = wpi::log::DoubleLogEntry(log, "/Chassis/BackRightModule/Drive/Power");
    // m_rbDriveEnergy = wpi::log::DoubleLogEntry(log, "/Chassis/BackRightModule/Drive/Energy");
    // m_rbDriveTotalPower = wpi::log::DoubleLogEntry(log, "/Chassis/BackRightModule/Drive/TotalPower");
    // m_rbDriveWattHours = wpi::log::DoubleLogEntry(log, "/Chassis/BackRightModule/Drive/WattHours");

    // m_swerveChassisTotalPower = wpi::log::DoubleLogEntry(log, "/Chassis/TotalPower");
    // m_swerveChassisWattHours = wpi::log::DoubleLogEntry(log, "/Chassis/WattHours");

    // // electrical signals
    // m_electricalVoltage = wpi::log::DoubleLogEntry(log, "/Electrical/Voltage(Volts)");
    // m_electricalVoltage.Append(0.0);
    // m_electricalCurrent = wpi::log::DoubleLogEntry(log, "/Electrical/Current(Amps)");
    // m_electricalCurrent.Append(0.0);
    // m_electricalPower = wpi::log::DoubleLogEntry(log, "/Electrical/Power(Watts)");
    // m_electricalPower.Append(0.0);
    // m_electricalEnergy = wpi::log::DoubleLogEntry(log, "/Electrical/Energy(Joules)");
    // m_electricalEnergy.Append(0.0);

    // m_headingState = wpi::log::StringLogEntry(log, "/Chassis/HeadingState");
    // m_headingState.Append("NONE");
    // m_driveState = wpi::log::StringLogEntry(log, "/Chassis/DriveState");
    // m_driveState.Append("NONE");

    // m_pose2d = wpi::log::StructLogEntry<frc::Pose2d>(log, "/Robot/Pose2d");
    // m_pose2d.Append(frc::Pose2d());

    // m_visionLeftReefBranchPose = wpi::log::StructLogEntry<frc::Pose2d>(log, "/Robot/VisionLeftReefBranchPose");
    // m_visionRightReefBranchPose = wpi::log::StructLogEntry<frc::Pose2d>(log, "/Robot/VisionRightReefBranchPose");
    // m_visionCoralStationPose = wpi::log::StructLogEntry<frc::Pose2d>(log, "/Robot/VisionCoralStationPose");
    // m_odometryLeftReefBranchPose = wpi::log::StructLogEntry<frc::Pose2d>(log, "/Robot/OdometryLeftReefBranchPose");
    // m_odometryRightReefBranchPose = wpi::log::StructLogEntry<frc::Pose2d>(log, "/Robot/OdometryRightReefBranchPose");
    // m_odometryCoralStationPose = wpi::log::StructLogEntry<frc::Pose2d>(log, "/Robot/OdometryCoralStationPose");

    // m_pose3dLimelight = wpi::log::StructLogEntry<frc::Pose3d>(log, "/Robot/Pose3dLimelight");
    // m_pose3dLimelight.Append(frc::Pose3d());

    // m_pose3dLimelight2 = wpi::log::StructLogEntry<frc::Pose3d>(log, "/Robot/Pose3dLimelight2");
    // m_pose3dLimelight2.Append(frc::Pose3d());

    // m_pose3dQuest = wpi::log::StructLogEntry<frc::Pose3d>(log, "/Robot/Pose3dQuest");
    // m_pose3dQuest.Append(frc::Pose3d());

    // m_frontLeftTarget = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/FrontLeftModule/TargetState");
    // m_frontLeftTarget.Append(frc::SwerveModuleState());
    // m_frontRightTarget = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/FrontRighttModule/TargetState");
    // m_frontRightTarget.Append(frc::SwerveModuleState());
    // m_backLeftTarget = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/BackLeftModule/TargetState");
    // m_backLeftTarget.Append(frc::SwerveModuleState());
    // m_backRightTarget = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/BackRightModule/TargetState");
    // m_backRightTarget.Append(frc::SwerveModuleState());
    // m_frontLeftActual = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/FrontLeftModule/ActualState");
    // m_frontLeftActual.Append(frc::SwerveModuleState());
    // m_frontRightActual = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/FrontRighttModule/ActualState");
    // m_frontRightActual.Append(frc::SwerveModuleState());
    // m_backLeftActual = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/BackLeftModule/ActualState");
    // m_backLeftActual.Append(frc::SwerveModuleState());
    // m_backRightActual = wpi::log::StructLogEntry<frc::SwerveModuleState>(log, "/Chassis/BackRightModule/ActualState");
    // m_backRightActual.Append(frc::SwerveModuleState());

    // m_actualSpeeds = wpi::log::StructLogEntry<frc::ChassisSpeeds>(log, "/Chassis/ActualSpeed");
    // m_actualSpeeds.Append(frc::ChassisSpeeds());
    // m_targetSpeeds = wpi::log::StructLogEntry<frc::ChassisSpeeds>(log, "/Chassis/TargetSpeed");
    // m_targetSpeeds.Append(frc::ChassisSpeeds());
}