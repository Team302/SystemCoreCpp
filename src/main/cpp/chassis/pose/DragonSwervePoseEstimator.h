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

#include <vector>

#include "chassis/pose/DragonVisionPoseEstimator.h"
#include "chassis/SwerveModule.h"
#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/SwerveDriveKinematics.h"

class SwerveChassis;

class DragonSwervePoseEstimator
{
public:
    DragonSwervePoseEstimator(frc::SwerveDriveKinematics<4> kinematics,
                              const frc::Rotation2d &gyroAngle,
                              const wpi::array<frc::SwerveModulePosition, 4> &positions,
                              const frc::Pose2d &initialPose);

    void RegisterVisionPoseEstimator(DragonVisionPoseEstimator *poseEstimator);

    void Update();
    frc::Pose2d GetPose() const;
    void ResetPosition(const frc::Pose2d &pose);
    void ResetPose(const frc::Pose2d &pose);
    void ZeroYaw();
    void CalculateInitialPose();
    std::vector<DragonVisionPoseEstimator *> GetVisionPoseEstimators() { return m_visionPoseEstimators; };

private:
    void AddVisionMeasurements();
    DragonSwervePoseEstimator() = delete;
    ~DragonSwervePoseEstimator() = default;

    DragonSwervePoseEstimator(const DragonSwervePoseEstimator &) = delete;
    DragonSwervePoseEstimator &operator=(const DragonSwervePoseEstimator &) = delete;

    void SetServeModules(SwerveChassis *chassis);

    SwerveModule *m_frontLeft;
    SwerveModule *m_frontRight;
    SwerveModule *m_backLeft;
    SwerveModule *m_backRight;
    frc::SwerveDriveKinematics<4> m_kinematics;
    frc::SwerveDrivePoseEstimator<4> m_poseEstimator;

    std::vector<DragonVisionPoseEstimator *> m_visionPoseEstimators;
};
