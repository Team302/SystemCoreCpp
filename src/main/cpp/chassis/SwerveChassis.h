
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
#include <map>
#include <string>

#include "chassis/ChassisMovement.h"
#include "chassis/ChassisOptionEnums.h"
#include "chassis/IChassis.h"
#include "chassis/pose/DragonSwervePoseEstimator.h"
#include "chassis/states/ISwerveDriveOrientation.h"
#include "chassis/states/ISwerveDriveState.h"
#include "chassis/SwerveModule.h"
#include "ctre/phoenix6/Pigeon2.hpp"
#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "pathplanner/lib/config/RobotConfig.h"
#include "RobinHood/robin_hood.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/mass.h"
#include "units/moment_of_inertia.h"
#include "units/velocity.h"
#include "utils/logging/signals/DragonDataLogger.h"

class RobotDrive;

class SwerveChassis : public IChassis, public DragonDataLogger
{
public:
    /// @brief Construct a swerve chassis
    SwerveChassis(SwerveModule *frontLeft,
                  SwerveModule *frontRight,
                  SwerveModule *backLeft,
                  SwerveModule *backRight,
                  ctre::phoenix6::hardware::Pigeon2 *pigeon,
                  std::string networkTableName,
                  units::length::inch_t wheelBase,
                  units::length::inch_t wheelTrack,
                  units::length::inch_t wheelDiameter,
                  units::velocity::feet_per_second_t maxSpeed);

    ~SwerveChassis() noexcept override = default;

    enum SWERVE_MODULES
    {
        LEFT_FRONT,
        RIGHT_FRONT,
        LEFT_BACK,
        RIGHT_BACK,
    };

    void InitStates();

    /// @brief Align all of the swerve modules to point forward
    void ZeroAlignSwerveModules();

    /// @brief Drive the chassis
    void Drive(ChassisMovement &moveInfo) override;

    /// @brief Provide the current chassis speed information
    frc::ChassisSpeeds GetChassisSpeeds() const;

    /// @brief Get encoder values
    double GetEncoderValues(SwerveModule *motor);

    /// @brief Reset the current chassis pose based on the provided pose (the rotation comes from the Pigeon)
    /// @param [in] const Pose2d&       pose        Current XY position
    void ResetPose(const frc::Pose2d &pose) override;

    /// @brief Reset yaw to 0 or 180 degrees depending on alliance
    void ResetYaw();
    void SetYaw(units::angle::degree_t newYaw);

    void LogSwerveEncoderData(SwerveChassis::SWERVE_MODULES swerveModule);

    units::length::inch_t GetWheelDiameter() const override;
    units::length::inch_t GetWheelBase() const { return m_wheelBase; }
    units::length::inch_t GetTrack() const { return m_track; }
    units::velocity::meters_per_second_t GetMaxSpeed() const override;
    units::angular_velocity::radians_per_second_t GetMaxAngularSpeed() const override;

    SwerveModule *GetFrontLeft() const { return m_frontLeft; }
    SwerveModule *GetFrontRight() const { return m_frontRight; }
    SwerveModule *GetBackLeft() const { return m_backLeft; }
    SwerveModule *GetBackRight() const { return m_backRight; }
    frc::Pose2d GetPose() const;
    units::angle::degree_t GetYaw() const override;
    units::angle::degree_t GetRawYaw();

    units::angle::degree_t GetPitch() const;
    units::angle::degree_t GetRoll() const;

    frc::SwerveDriveKinematics<4> GetKinematics() const { return m_kinematics; }

    // Dummy functions for IChassis Implementation
    inline IChassis::CHASSIS_TYPE GetType() const override { return IChassis::CHASSIS_TYPE::SWERVE; };
    inline void Initialize() override {};

    void SetTargetHeading(units::angle::degree_t targetYaw) override;

    void SetStoredHeading(units::angle::degree_t heading);
    units::angle::degree_t GetStoredHeading() const { return m_storedYaw; };

    ISwerveDriveOrientation *GetSpecifiedHeadingState(ChassisOptionEnums::HeadingOption headingOption);
    ISwerveDriveState *GetSpecifiedDriveState(ChassisOptionEnums::DriveStateType driveOption);

    bool IsRotating() const { return m_rotatingLatch; }
    bool IsSamePose();
    bool IsSamePose(units::length::inch_t distanceThreshold);
    double GetRotationRateDegreesPerSecond() const { return m_pigeon != nullptr ? m_pigeon->GetAngularVelocityZWorld(true).GetValueAsDouble() : 0.0; }
    void LogInformation();
    void DataLog(uint64_t timestamp) override;

    units::mass::kilogram_t GetMass() const { return m_mass; }
    units::moment_of_inertia::kilogram_square_meter_t GetMomenOfInertia() const { return m_momentOfInertia; }

    frc::Translation2d GetFrontLeftOffset() const { return m_frontLeftLocation; }
    frc::Translation2d GetFrontRightOffset() const { return m_frontRightLocation; }
    frc::Translation2d GetBackLeftOffset() const { return m_backLeftLocation; }
    frc::Translation2d GetBackRightOffset() const { return m_backRightLocation; }

    DragonSwervePoseEstimator *GetSwervePoseEstimator() { return m_swervePoseEstimator; }

private:
    int m_samePoseCount;
    const int m_samePoseCountThreshold = 25;
    const units::length::inch_t m_distanceThreshold{0.25};
    frc::Pose2d m_prevPose;

    ISwerveDriveOrientation *GetHeadingState(const ChassisMovement &moveInfo);
    ISwerveDriveState *GetDriveState(ChassisMovement &moveInfo);

    SwerveModule *m_frontLeft = nullptr;
    SwerveModule *m_frontRight = nullptr;
    SwerveModule *m_backLeft = nullptr;
    SwerveModule *m_backRight = nullptr;
    ctre::phoenix6::hardware::Pigeon2 *m_pigeon = nullptr;

    RobotDrive *m_robotDrive = nullptr;
    robin_hood::unordered_map<ChassisOptionEnums::DriveStateType, ISwerveDriveState *> m_driveStateMap;
    robin_hood::unordered_map<ChassisOptionEnums::HeadingOption, ISwerveDriveOrientation *> m_headingStateMap;

    units::length::inch_t m_wheelBase = units::length::inch_t(22.75);
    units::length::inch_t m_track = units::length::inch_t(22.75);
    units::velocity::feet_per_second_t m_maxSpeed = units::velocity::feet_per_second_t(17.3);
    units::length::inch_t m_wheelDiameter = units::length::inch_t(4.0);
    units::length::meter_t m_radius;

    units::velocity::meters_per_second_t m_drive = units::velocity::meters_per_second_t(0.0);
    units::velocity::meters_per_second_t m_steer = units::velocity::meters_per_second_t(0.0);
    units::angular_velocity::radians_per_second_t m_rotate = units::angular_velocity::radians_per_second_t(0.0);

    static constexpr units::angular_velocity::radians_per_second_t m_angularDeadband = units::angular_velocity::radians_per_second_t(0.1);
    static constexpr units::acceleration::meters_per_second_squared_t m_accelerationThreshold = 0.5_mps_sq;
    static constexpr units::velocity::meters_per_second_t m_velocityThresold = units::velocity::meters_per_second_t(0.25);

    frc::Translation2d m_frontLeftLocation;
    frc::Translation2d m_frontRightLocation;
    frc::Translation2d m_backLeftLocation;
    frc::Translation2d m_backRightLocation;
    frc::SwerveDriveKinematics<4> m_kinematics;
    DragonSwervePoseEstimator *m_swervePoseEstimator = nullptr;
    units::angle::degree_t m_storedYaw;
    units::angle::degree_t m_targetHeading;
    ISwerveDriveState *m_currentDriveState = nullptr;
    ISwerveDriveOrientation *m_currentOrientationState = nullptr;
    bool m_initialized = false;
    std::string m_networkTableName;
    bool m_rotatingLatch = false;
    bool m_initDataLog = false;
    double m_coseAngle = 0.707;
    std::array<frc::SwerveModuleState, 4U> m_targetStates;
    units::mass::kilogram_t m_mass = units::mass::kilogram_t(52.0);                                                                // TODO put a real value in
    units::moment_of_inertia::kilogram_square_meter_t m_momentOfInertia = units::moment_of_inertia::kilogram_square_meter_t(26.0); // TODO put a real value in
    frc::Timer m_velocityTimer;

    const units::angle::degree_t m_specifiedHeadingTolerance{0.5};
    units::angle::degree_t m_simRotation{0.0};

    double m_totalEnergy = 0.0;
    double m_totalWattHours = 0.0;
    frc::Timer m_powerTimer;
};