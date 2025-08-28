
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
#include <tuple>

#include "fielddata/FieldConstants.h"
#include "frc/geometry/Pose2d.h"
#include "units/angle.h"
#include "vision/DragonVision.h"
#include "utils/logging/signals/DragonDataLogger.h"

enum class DragonTargetFinderTarget
{
    ALGAE,
    BARGE,
    CLOSEST_LEFT_REEF_BRANCH,
    CLOSEST_RIGHT_REEF_BRANCH,
    CLOSEST_REEF_ALGAE,
    REEF_CENTER,
    CLOSEST_CORAL_STATION_SIDWALL_SIDE,
    CLOSEST_CORAL_STATION_MIDDLE,
    CLOSEST_CORAL_STATION_ALLIANCE_SIDE,
    LEFT_CAGE,
    CENTER_CAGE,
    RIGHT_CAGE,
    PROCESSOR
};

enum class DragonTargetFinderData
{
    NOT_FOUND,
    VISION_BASED = 1,
    ODOMETRY_BASED = 10,
    VISION_ODOMETRY_FUSED = 11
};

class DragonTargetFinder : public DragonDataLogger
{
public:
    void DataLog(uint64_t timestamp) override;

    static DragonTargetFinder *GetInstance();
    std::optional<std::tuple<DragonTargetFinderData, frc::Pose2d>> GetPose(DragonTargetFinderTarget item);

    void ResetGoalPose();

private:
    DragonTargetFinder();
    ~DragonTargetFinder() = default;
    static DragonTargetFinder *m_instance;

    subsystems::CommandSwerveDrivetrain *m_chassis;
    DragonVision *m_vision;
    DragonTargetFinderTarget m_targetVisionTarget;

    std::optional<FieldConstants::AprilTagIDs> GetAprilTag(DragonVision::VISION_ELEMENT item);
    frc::Pose3d GetAprilTagPose(DragonVision::VISION_ELEMENT item);
    units::angle::degree_t AdjustRobotRelativeAngleForIntake(units::angle::degree_t angle);
    std::optional<frc::Pose2d> GetFieldRelativePose(std::optional<VisionData> data);
    bool SwitchToVision(std::optional<frc::Pose3d> visTagPose);

    void SetChassis();

    std::optional<frc::Pose2d> m_goalPose;

    bool m_switchToVision = false;
    static constexpr units::length::meter_t m_fuseTol{0.25};
    static constexpr units::length::meter_t m_switchToVisionThreshold{1.0};
    static constexpr frc::Transform2d m_intakeOffset = frc::Transform2d{
        frc::Translation2d(12.0_in, 6.5_in),
        frc::Rotation2d(0_deg)};
};
