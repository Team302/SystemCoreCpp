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
#include <cmath>
#include <numbers>

// FRC includes
#include "frc/DriverStation.h"
#include "frc/Filesystem.h"
#include "frc/geometry/Rotation2d.h"
#include "units/angular_acceleration.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/DataLogManager.h"
#include "frc/Timer.h"
#include "frc/RobotBase.h"

// Team 302 includes
#include "chassis/states/FieldDrive.h"
#include "chassis/states/HoldDrive.h"
#include "chassis/states/RobotDrive.h"
#include "chassis/states/PolarDrive.h"
#include "chassis/states/StopDrive.h"
//#include "chassis/states/TrajectoryDrivePathPlanner.h"
#include "chassis/states/IgnoreHeading.h"
#include "chassis/states/ISwerveDriveOrientation.h"
#include "chassis/states/MaintainHeading.h"
#include "chassis/states/SpecifiedHeading.h"
#include "chassis/states/FaceGamePiece.h"
#include "chassis/states/FaceReefCenter.h"
#include "chassis/states/FaceNearestReefFace.h"
#include "chassis/states/FaceNearestCoralStation.h"
#include "chassis/states/FaceBarge.h"
#include "chassis/states/DriveToCoralStation.h"
#include "chassis/states/DriveToBarge.h"
#include "chassis/states/DriveToRightReefBranch.h"
#include "chassis/LogChassisMovement.h"
#include "chassis/SwerveChassis.h"
#include "utils/DragonPower.h"
#include "utils/logging/debug/Logger.h"
#include "utils/AngleUtils.h"
#include "chassis/states/DriveToLeftReefBranch.h"
#include "chassis/states/DriveToLeftCage.h"
#include "chassis/states/DriveToRightCage.h"
#include "chassis/states/DriveToCenterCage.h"
#include "chassis/states/DriveToAlgae.h"
#include "chassis/states/DriveToProcessor.h"

#include "state/RobotState.h"

// Third Party Includes
#include "pugixml/pugixml.hpp"
#include <ctre/phoenix6/StatusSignal.hpp>
#include "ctre/phoenix6/SignalLogger.hpp"

using robin_hood::unordered_map;
using std::string;

using frc::ChassisSpeeds;
using frc::Pose2d;
using frc::Rotation2d;
using frc::SwerveModulePosition;

using ctre::phoenix6::SignalLogger;
using ctre::phoenix6::configs::MountPoseConfigs;
using ctre::phoenix6::hardware::Pigeon2;

/// @brief Construct a swerve chassis
SwerveChassis::SwerveChassis(SwerveModule *frontLeft,
                             SwerveModule *frontRight,
                             SwerveModule *backLeft,
                             SwerveModule *backRight,
                             Pigeon2 *pigeon,
                             string networkTableName,
                             units::length::inch_t wheelBase,
                             units::length::inch_t wheelTrack,
                             units::length::inch_t wheelDiameter,
                             units::velocity::feet_per_second_t maxSpeed) : IChassis(),
                                                                            m_frontLeft(frontLeft),
                                                                            m_frontRight(frontRight),
                                                                            m_backLeft(backLeft),
                                                                            m_backRight(backRight),
                                                                            m_pigeon(pigeon),
                                                                            m_robotDrive(nullptr),
                                                                            m_wheelBase(wheelBase),
                                                                            m_track(wheelTrack),
                                                                            m_maxSpeed(maxSpeed),
                                                                            m_wheelDiameter(wheelDiameter),
                                                                            m_frontLeftLocation(wheelBase / 2.0, wheelTrack / 2.0),
                                                                            m_frontRightLocation(wheelBase / 2.0, -wheelTrack / 2.0),
                                                                            m_backLeftLocation(-wheelBase / 2.0, wheelTrack / 2.0),
                                                                            m_backRightLocation(-wheelBase / 2.0, -wheelTrack / 2.0),
                                                                            m_kinematics(m_frontLeftLocation,
                                                                                         m_frontRightLocation,
                                                                                         m_backLeftLocation,
                                                                                         m_backRightLocation),
                                                                            m_swervePoseEstimator(nullptr),
                                                                            m_storedYaw(units::angle::degree_t(0.0)),
                                                                            m_targetHeading(units::angle::degree_t(0.0)),
                                                                            m_networkTableName(networkTableName)
{
    InitStates();
    m_pigeon->Reset();
    ResetPose(frc::Pose2d());
    m_maxSpeed = m_frontLeft->GetMaxSpeed();
    m_radius = m_frontLeftLocation.Norm();

    ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(100_Hz, m_pigeon->GetYaw(), m_pigeon->GetPitch(), m_pigeon->GetRoll(), m_pigeon->GetAccelerationX(), m_pigeon->GetAccelerationY());

    m_swervePoseEstimator = new DragonSwervePoseEstimator(m_kinematics,
                                                          Rotation2d(),
                                                          {SwerveModulePosition(),
                                                           SwerveModulePosition(),
                                                           SwerveModulePosition(),
                                                           SwerveModulePosition()},
                                                          Pose2d());


    std::vector<frc::Translation2d> moduleLocs;
    moduleLocs.emplace_back(GetFrontLeftOffset());
    moduleLocs.emplace_back(GetFrontRightOffset());
    moduleLocs.emplace_back(GetBackLeftOffset());
    moduleLocs.emplace_back(GetBackRightOffset());

}

//==================================================================================
void SwerveChassis::InitStates()
{
    m_robotDrive = new RobotDrive(this);

    m_driveStateMap[ChassisOptionEnums::DriveStateType::FIELD_DRIVE] = new FieldDrive(m_robotDrive);
    m_driveStateMap[ChassisOptionEnums::DriveStateType::POLAR_DRIVE] = new PolarDrive(m_robotDrive);
    m_driveStateMap[ChassisOptionEnums::DriveStateType::HOLD_DRIVE] = new HoldDrive();
    m_driveStateMap[ChassisOptionEnums::DriveStateType::ROBOT_DRIVE] = m_robotDrive;
    m_driveStateMap[ChassisOptionEnums::DriveStateType::STOP_DRIVE] = new StopDrive(m_robotDrive);
    m_driveStateMap[ChassisOptionEnums::DriveStateType::DRIVE_TO_CORAL_STATION] = new DriveToCoralStation(m_robotDrive);
    m_driveStateMap[ChassisOptionEnums::DriveStateType::DRIVE_TO_LEFT_REEF_BRANCH] = new DriveToLeftReefBranch(m_robotDrive);
    m_driveStateMap[ChassisOptionEnums::DriveStateType::DRIVE_TO_RIGHT_REEF_BRANCH] = new DriveToRightReefBranch(m_robotDrive);
    m_driveStateMap[ChassisOptionEnums::DriveStateType::DRIVE_TO_RIGHT_CAGE] = new DriveToRightCage(m_robotDrive);
    m_driveStateMap[ChassisOptionEnums::DriveStateType::DRIVE_TO_LEFT_CAGE] = new DriveToLeftCage(m_robotDrive);
    m_driveStateMap[ChassisOptionEnums::DriveStateType::DRIVE_TO_CENTER_CAGE] = new DriveToCenterCage(m_robotDrive);
    m_driveStateMap[ChassisOptionEnums::DriveStateType::DRIVE_TO_BARGE] = new DriveToBarge(m_robotDrive);
    m_driveStateMap[ChassisOptionEnums::DriveStateType::DRIVE_TO_ALGAE] = new DriveToAlgae(m_robotDrive);
    m_driveStateMap[ChassisOptionEnums::DriveStateType::DRIVE_TO_PROCESSOR] = new DriveToProcessor(m_robotDrive);

    m_headingStateMap[ChassisOptionEnums::HeadingOption::MAINTAIN] = new MaintainHeading();
    m_headingStateMap[ChassisOptionEnums::HeadingOption::SPECIFIED_ANGLE] = new SpecifiedHeading();
    m_headingStateMap[ChassisOptionEnums::HeadingOption::FACE_GAME_PIECE] = new FaceGamePiece();
    m_headingStateMap[ChassisOptionEnums::HeadingOption::FACE_REEF_CENTER] = new FaceReefCenter();
    m_headingStateMap[ChassisOptionEnums::HeadingOption::FACE_REEF_FACE] = new FaceNearestReefFace();
    m_headingStateMap[ChassisOptionEnums::HeadingOption::FACE_CORAL_STATION] = new FaceNearestCoralStation();
    m_headingStateMap[ChassisOptionEnums::HeadingOption::FACE_BARGE] = new FaceBarge();
    m_headingStateMap[ChassisOptionEnums::HeadingOption::IGNORE] = new IgnoreHeading();
}

//==================================================================================
/// @brief Align all of the swerve modules to point forward
void SwerveChassis::ZeroAlignSwerveModules()
{
    m_frontLeft->ZeroAlignModule();
    m_frontRight->ZeroAlignModule();
    m_backLeft->ZeroAlignModule();
    m_backRight->ZeroAlignModule();
}

//==================================================================================
/// @brief Drive the chassis
void SwerveChassis::Drive(ChassisMovement &moveInfo)
{

    if (abs(moveInfo.rawOmega) > 0.05)
    {
        m_rotatingLatch = true;
    }
    else if (abs(GetRotationRateDegreesPerSecond()) < 5.0) // degrees per second
    {
        m_rotatingLatch = false;
    }

    m_currentOrientationState = GetHeadingState(moveInfo);
    if (m_currentOrientationState != nullptr)
    {
        m_currentOrientationState->UpdateChassisSpeeds(moveInfo);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_networkTableName, string("Heading Option"), -1);
    }

    m_currentDriveState = GetDriveState(moveInfo);
    if (m_currentDriveState != nullptr)
    {
        m_targetStates = m_currentDriveState->UpdateSwerveModuleStates(moveInfo);
        RobotState::GetInstance()->PublishStateChange(RobotStateChanges::DriveStateType_Int, moveInfo.driveOption);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_networkTableName, string("Drive Option"), string("NONE"));
    }

    // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_networkTableName, string("Drive Option"), moveInfo.driveOption);
    // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_networkTableName, string("Heading Option"), moveInfo.headingOption);
    m_drive = moveInfo.chassisSpeeds.vx;
    m_steer = moveInfo.chassisSpeeds.vy;
    m_rotate = moveInfo.chassisSpeeds.omega;

    m_frontLeft->SetDesiredState(m_targetStates[LEFT_FRONT]);
    m_frontRight->SetDesiredState(m_targetStates[RIGHT_FRONT]);
    m_backLeft->SetDesiredState(m_targetStates[LEFT_BACK]);
    m_backRight->SetDesiredState(m_targetStates[RIGHT_BACK]);
    LogInformation();
    // m_rotate = moveInfo.chassisSpeeds.omega TO DO this is in place for Data Logging, need to create dataLog method where we pass moveInfo to it and it handels the data logging variables
}

//==================================================================================
ISwerveDriveState *SwerveChassis::GetSpecifiedDriveState(ChassisOptionEnums::DriveStateType driveOption)
{
    auto itr = m_driveStateMap.find(driveOption);
    if (itr == m_driveStateMap.end())
    {
        return m_robotDrive;
    }
    return itr->second;
}

//==================================================================================
ISwerveDriveOrientation *SwerveChassis::GetSpecifiedHeadingState(ChassisOptionEnums::HeadingOption headingOption)
{
    auto itr = m_headingStateMap.find(headingOption);
    if (itr == m_headingStateMap.end())
    {
        return m_headingStateMap[ChassisOptionEnums::HeadingOption::MAINTAIN];
    }
    return itr->second;
}

//==================================================================================
ISwerveDriveOrientation *SwerveChassis::GetHeadingState(const ChassisMovement &moveInfo)
{
    auto itr = m_headingStateMap.find(moveInfo.headingOption);
    if (itr == m_headingStateMap.end())
    {
        itr = m_headingStateMap.find(ChassisOptionEnums::HeadingOption::MAINTAIN);
    }
    return itr->second;
}

//==================================================================================
ISwerveDriveState *SwerveChassis::GetDriveState(ChassisMovement &moveInfo)
{
    auto state = GetSpecifiedDriveState(moveInfo.driveOption);

    SignalLogger::WriteDouble(std::string("/swervechassis/state"), static_cast<int>(moveInfo.driveOption), "", units::time::second_t((0)));

    auto itr = m_driveStateMap.find(moveInfo.driveOption);
    if (itr == m_driveStateMap.end())
    {
        return m_robotDrive;
    }
    state = itr->second;

    if (m_currentDriveState == nullptr)
    {
        m_currentDriveState = m_robotDrive;
    }

    if (state != m_currentDriveState)
    {
        m_initialized = false;
    }

    if (!m_initialized && state != nullptr)
    {
        // not sure if this is the right spot, but it should work for now.
        m_samePoseCount = 0;

        state->Init(moveInfo);
        m_initialized = true;
        m_currentDriveState = state;
    }

    return state;
}

//==================================================================================
Pose2d SwerveChassis::GetPose() const
{
    if (m_swervePoseEstimator != nullptr)
    {
        return m_swervePoseEstimator->GetPose();
    }
    return Pose2d();
}

//==================================================================================
units::angle::degree_t SwerveChassis::GetYaw() const
{
    return m_swervePoseEstimator->GetPose().Rotation().Degrees();
}

//==================================================================================
units::angle::degree_t SwerveChassis::GetRawYaw()
{
    if (frc::RobotBase::IsSimulation())
    {
        m_simRotation = m_simRotation + (m_rotate * 0.02_s);
        return m_simRotation;
    }
    else
    {
        return m_pigeon->GetYaw().Refresh().GetValue();
    }
}

//==================================================================================
units::angle::degree_t SwerveChassis::GetPitch() const
{
    return m_pigeon->GetPitch().GetValue();
}

//==================================================================================
units::angle::degree_t SwerveChassis::GetRoll() const
{
    return m_pigeon->GetRoll().GetValue();
}

//==================================================================================
double SwerveChassis::GetEncoderValues(SwerveModule *motor)
{
    return motor->GetEncoderValues();
}

//==================================================================================
/// @brief Provide the current chassis speed information
ChassisSpeeds SwerveChassis::GetChassisSpeeds() const
{
    return m_kinematics.ToChassisSpeeds({m_frontLeft->GetState(),
                                         m_frontRight->GetState(),
                                         m_backLeft->GetState(),
                                         m_backRight->GetState()});
}
//==================================================================================

void SwerveChassis::LogSwerveEncoderData(SwerveChassis::SWERVE_MODULES swerveModule)
{
    if (swerveModule == SwerveChassis::SWERVE_MODULES::RIGHT_FRONT)
    {
        m_frontRight->LogInformation();
    }
    else if (swerveModule == SwerveChassis::SWERVE_MODULES::RIGHT_BACK)
    {
        m_backRight->LogInformation();
    }
    else if (swerveModule == SwerveChassis::SWERVE_MODULES::LEFT_FRONT)
    {
        m_frontLeft->LogInformation();
    }
    else if (swerveModule == SwerveChassis::SWERVE_MODULES::LEFT_BACK)
    {
        m_backLeft->LogInformation();
    }
}

//==================================================================================
void SwerveChassis::ResetPose(const Pose2d &pose)
{
    ZeroAlignSwerveModules();
    SetStoredHeading(pose.Rotation().Degrees());

    if (m_swervePoseEstimator != nullptr)
    {
        m_swervePoseEstimator->ResetPose(pose);
    }
}
//=================================================================================
void SwerveChassis::SetYaw(units::angle::degree_t newYaw)
{
    m_pigeon->SetYaw(newYaw);
    SetStoredHeading(newYaw);
}

//==================================================================================
void SwerveChassis::ResetYaw()
{
    m_pigeon->Reset();
    SetStoredHeading(units::angle::degree_t(0.0));
    ZeroAlignSwerveModules();
}

//==================================================================================
void SwerveChassis::SetStoredHeading(units::angle::degree_t heading)
{
    if (units::math::abs(heading - m_storedYaw) > m_specifiedHeadingTolerance)
    {
        m_storedYaw = heading;
    }
}

//==================================================================================
void SwerveChassis::SetTargetHeading(units::angle::degree_t targetYaw)
{
    m_targetHeading = targetYaw;
}

//==================================================================================
units::length::inch_t SwerveChassis::GetWheelDiameter() const
{
    return m_wheelDiameter;
}

//==================================================================================
units::velocity::meters_per_second_t SwerveChassis::GetMaxSpeed() const
{
    return m_maxSpeed;
}

//==================================================================================
units::angular_velocity::radians_per_second_t SwerveChassis::GetMaxAngularSpeed() const
{
    units::length::meter_t circumference = std::numbers::pi * m_wheelBase * m_coseAngle;
    auto angSpeed = units::angular_velocity::turns_per_second_t(GetMaxSpeed().to<double>() / circumference.to<double>());
    units::angular_velocity::radians_per_second_t retval = angSpeed;
    return retval;
}

//==================================================================================
void SwerveChassis::LogInformation()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_networkTableName, string("Vx"), m_drive.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_networkTableName, string("Vy"), m_steer.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_networkTableName, string("Omega"), m_rotate.to<double>());
    auto pose = GetPose();
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_networkTableName, string("current x position"), pose.X().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_networkTableName, string("current y position"), pose.Y().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_networkTableName, string("current rotation position"), pose.Rotation().Degrees().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_networkTableName, string("Stored Heading"), GetStoredHeading().value());
}
//==================================================================================
bool SwerveChassis::IsSamePose()
{
    bool isSamePose = false;

    if (GetPose().Translation().Distance(m_prevPose.Translation()) < m_distanceThreshold)
    {
        m_samePoseCount++;
        isSamePose = m_samePoseCount > m_samePoseCountThreshold;
    }
    else
    {
        m_samePoseCount = 0;
    }

    m_prevPose = GetPose();

    return isSamePose;
}
bool SwerveChassis::IsSamePose(units::length::inch_t distanceThreshold)
{
    bool isSamePose = false;

    if (GetPose().Translation().Distance(m_prevPose.Translation()) < distanceThreshold)
    {
        m_samePoseCount++;
        isSamePose = m_samePoseCount > m_samePoseCountThreshold;
    }
    else
    {
        m_samePoseCount = 0;
    }

    m_prevPose = GetPose();

    return isSamePose;
}
//==================================================================================

void SwerveChassis::DataLog(uint64_t timestamp)
{
    Log2DPoseData(timestamp, DragonDataLogger::PoseSingals::CURRENT_CHASSIS_POSE2D, GetPose());

    LogDoubleData(timestamp, DragonDataLogger::DoubleSignals::CHASSIS_STORED_HEADING_DEGREES, GetStoredHeading().value());
    LogDoubleData(timestamp, DragonDataLogger::DoubleSignals::CHASSIS_YAW_DEGREES, AngleUtils::GetEquivAngle(GetYaw()).value());

    frc::ChassisSpeeds targetSpeed;
    targetSpeed.vx = m_drive;
    targetSpeed.vy = m_steer;
    targetSpeed.omega = m_rotate;
    LogChassisSpeedsData(timestamp, DragonDataLogger::ChassisSpeedSignals::TARGET_SPEEDS, targetSpeed);

    auto currFrontLeftState = m_frontLeft->GetState();
    auto currFrontRightState = m_frontRight->GetState();
    auto currBackLeftState = m_backLeft->GetState();
    auto currBackRightState = m_backRight->GetState();
    wpi::array<frc::SwerveModuleState, 4> states = {currFrontLeftState, currFrontRightState, currBackLeftState, currBackRightState};
    auto currentSpeed = m_kinematics.ToChassisSpeeds(states);
    LogChassisSpeedsData(timestamp, DragonDataLogger::ChassisSpeedSignals::ACTUAL_SPEEDS, currentSpeed);

    auto optFrontLeftState = m_frontLeft->GetOptimizedState();
    auto optFrontRightState = m_frontRight->GetOptimizedState();
    auto optBackLeftState = m_backLeft->GetOptimizedState();
    auto optBackRightState = m_backRight->GetOptimizedState();
    LogSwerveModuleStateData(timestamp, DragonDataLogger::SwerveStateSingals::TARGET_LEFT_FRONT_STATE, optFrontLeftState);
    LogSwerveModuleStateData(timestamp, DragonDataLogger::SwerveStateSingals::TARGET_LEFT_BACK_STATE, optBackLeftState);
    LogSwerveModuleStateData(timestamp, DragonDataLogger::SwerveStateSingals::TARGET_RIGHT_FRONT_STATE, optFrontRightState);
    LogSwerveModuleStateData(timestamp, DragonDataLogger::SwerveStateSingals::TARGET_RIGHT_BACK_STATE, optBackRightState);

    LogSwerveModuleStateData(timestamp, DragonDataLogger::SwerveStateSingals::ACTUAL_LEFT_FRONT_STATE, currFrontLeftState);
    LogSwerveModuleStateData(timestamp, DragonDataLogger::SwerveStateSingals::ACTUAL_LEFT_BACK_STATE, currBackLeftState);
    LogSwerveModuleStateData(timestamp, DragonDataLogger::SwerveStateSingals::ACTUAL_RIGHT_FRONT_STATE, currFrontRightState);
    LogSwerveModuleStateData(timestamp, DragonDataLogger::SwerveStateSingals::ACTUAL_RIGHT_BACK_STATE, currBackRightState);

    if (m_currentDriveState != nullptr)
    {
        LogStringData(timestamp, DragonDataLogger::StringSignals::CHASSIS_DRIVE_STATE, m_currentDriveState->GetDriveStateName());
    }
    if (m_currentOrientationState != nullptr)
    {
        LogStringData(timestamp, DragonDataLogger::StringSignals::CHASSIS_HEADING_STATE, m_currentOrientationState->GetHeadingStateName());
    }

    auto currTime = m_powerTimer.Get();

    auto leftFrontSteerPower = m_frontLeft->CalcSteerPowerEnergy(currTime);
    auto power = get<0>(leftFrontSteerPower);
    auto energy = get<1>(leftFrontSteerPower);
    m_totalEnergy += energy;
    LogDoubleData(timestamp, DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_STEER_POWER, power);
    LogDoubleData(timestamp, DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_STEER_CURRENT, m_backLeft->GetSteerCurrent().value());

    auto leftFrontDrivePower = m_frontLeft->CalcDrivePowerEnergy(currTime);
    power = get<0>(leftFrontDrivePower);
    energy = get<1>(leftFrontDrivePower);
    m_totalEnergy += energy;
    LogDoubleData(timestamp, DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_DRIVE_POWER, power);
    LogDoubleData(timestamp, DragonDataLogger::DoubleSignals::LEFT_FRONT_SWERVE_DRIVE_CURRENT, m_frontLeft->GetDriveCurrent().value());

    auto leftBackSteerPower = m_backLeft->CalcSteerPowerEnergy(currTime);
    power = get<0>(leftBackSteerPower);
    energy = get<1>(leftBackSteerPower);
    m_totalEnergy += energy;
    LogDoubleData(timestamp, DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_STEER_POWER, power);
    LogDoubleData(timestamp, DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_STEER_CURRENT, m_backLeft->GetSteerCurrent().value());

    auto leftBackDrivePower = m_backLeft->CalcDrivePowerEnergy(currTime);
    power = get<0>(leftBackDrivePower);
    energy = get<1>(leftBackDrivePower);
    m_totalEnergy += energy;
    LogDoubleData(timestamp, DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_DRIVE_POWER, power);
    LogDoubleData(timestamp, DragonDataLogger::DoubleSignals::LEFT_BACK_SWERVE_DRIVE_CURRENT, m_backLeft->GetDriveCurrent().value());

    auto rightFrontSteerPower = m_frontRight->CalcSteerPowerEnergy(currTime);
    power = get<0>(rightFrontSteerPower);
    energy = get<1>(rightFrontSteerPower);
    m_totalEnergy += energy;
    LogDoubleData(timestamp, DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_STEER_POWER, power);
    LogDoubleData(timestamp, DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_STEER_CURRENT, m_frontRight->GetSteerCurrent().value());

    auto rightFrontDrivePower = m_frontRight->CalcDrivePowerEnergy(currTime);
    power = get<0>(rightFrontDrivePower);
    energy = get<1>(rightFrontDrivePower);
    m_totalEnergy += energy;
    LogDoubleData(timestamp, DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_DRIVE_POWER, power);
    LogDoubleData(timestamp, DragonDataLogger::DoubleSignals::RIGHT_FRONT_SWERVE_DRIVE_CURRENT, m_frontRight->GetDriveCurrent().value());

    auto rightBackSteerPower = m_backRight->CalcSteerPowerEnergy(currTime);
    power = get<0>(rightBackSteerPower);
    energy = get<1>(rightBackSteerPower);
    m_totalEnergy += energy;
    LogDoubleData(timestamp, DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_STEER_POWER, power);
    LogDoubleData(timestamp, DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_STEER_CURRENT, m_backRight->GetSteerCurrent().value());

    auto rightBackDrivePower = m_backRight->CalcDrivePowerEnergy(currTime);
    power = get<0>(rightBackDrivePower);
    energy = get<1>(rightBackDrivePower);
    m_totalEnergy += energy;
    LogDoubleData(timestamp, DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_DRIVE_POWER, power);
    LogDoubleData(timestamp, DragonDataLogger::DoubleSignals::RIGHT_BACK_SWERVE_DRIVE_CURRENT, m_backRight->GetDriveCurrent().value());

    m_totalWattHours += DragonPower::ConvertEnergyToWattHours(m_totalEnergy);
    LogDoubleData(timestamp, DragonDataLogger::DoubleSignals::SWERVE_CHASSIS_TOTAL_CURRENT, m_totalEnergy);
    LogDoubleData(timestamp, DragonDataLogger::DoubleSignals::SWERVE_CHASSIS_WATT_HOURS, m_totalWattHours);

    m_powerTimer.Reset();
    m_powerTimer.Start();
}
