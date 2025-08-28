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

#include "frc2/command/CommandHelper.h"
#include "frc2/command/Command.h"
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Pose2d.h>
#include "chassis/generated/CommandSwerveDrivetrain.h"
#include "fielddata/DragonTargetFinder.h"

class DriveToTarget : public frc2::CommandHelper<frc2::Command, DriveToTarget>
{
public:
    /**
     * @brief Creates a command to drive to a specified field element using vision and odometry.
     *
     * @param chassis A pointer to the swerve drive subsystem.
     * @param target The specific field element to target.
     */
    DriveToTarget(subsystems::CommandSwerveDrivetrain *chassis, DragonTargetFinderTarget target);

    // FRC Command Lifecycle methods
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

private:
    void CalculateFeedForward(frc::ChassisSpeeds &chassisSpeeds);

    subsystems::CommandSwerveDrivetrain *m_chassis;
    DragonTargetFinderTarget m_target;
    DragonTargetFinder *m_targetFinder;
    DragonTargetFinderData m_currentType = DragonTargetFinderData::NOT_FOUND;

    swerve::requests::FieldCentricFacingAngle m_driveRequest;

    bool m_hasTarget = false;
    bool m_isSamePose = false;
    frc::Pose2d m_endPose;
    frc::Pose2d m_prevPose;
    frc::Pose2d m_currentPose;
    const units::length::inch_t m_distanceThreshold{0.25};
    const units::length::inch_t m_regenerationDistanceThreshold{2.0};
    const units::length::meter_t m_ffMinRadius{0.0};
    const units::length::meter_t m_ffMaxRadius{1.25};

    const units::velocity::meters_per_second_t kMaxVelocity = 4_mps;
    const units::acceleration::meters_per_second_squared_t kMaxAcceleration = 4_mps_sq;

    const units::angular_velocity::degrees_per_second_t kMaxAngularVelocity = 540_deg_per_s;

    std::string m_iGainKey = "I_Gain";
    std::string m_pGainKey = "P_Gain";
    bool runOnceLatch = false;

    const double m_translationKP = 4.5;
    const double m_translationKI = 0.0;
    const double m_translationKD = 0.5;

    const double m_rotationKP = 6.0;
    const double m_rotationKI = 0.0;
    const double m_rotationKD = 0.0;

    units::angle::degree_t m_sweepDelta{90.0};

    units::length::meter_t m_distanceError{0.0};
    units::length::meter_t m_pidResetThreshold{0.25};
    units::length::meter_t m_feedForwardRange;

    frc::TrapezoidProfile<units::length::meters>::Constraints m_translationConstraints{kMaxVelocity, kMaxAcceleration};

    frc::ProfiledPIDController<units::length::meters> m_translationPIDX{m_translationKP, m_translationKI, m_translationKD, m_translationConstraints, 20_ms};
    frc::ProfiledPIDController<units::length::meters> m_translationPIDY{m_translationKP, m_translationKI, m_translationKD, m_translationConstraints, 20_ms};
};
