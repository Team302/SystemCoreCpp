
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

#include <optional>

#include "fielddata/FieldConstants.h"
#include "frc/DriverStation.h"
#include "frc/geometry/Pose2d.h"
#include "auton/ZoneParams.h"
#include "auton/ZoneParser.h"
#include "state/RobotState.h"
class ReefHelper
{
public:
    static ReefHelper *GetInstance();
    void IsInZone();
    void InitZones();
    std::optional<FieldConstants::AprilTagIDs> GetNearestReefTag();
    std::optional<FieldConstants::FIELD_ELEMENT> GetNearestLeftReefBranch(FieldConstants::AprilTagIDs tag);
    std::optional<FieldConstants::FIELD_ELEMENT> GetNearestRightReefBranch(FieldConstants::AprilTagIDs tag);

private:
    ReefHelper();
    ~ReefHelper() = default;
    static ReefHelper *m_instance;

    units::length::meter_t CalcDistanceToAprilTag(FieldConstants::AprilTagIDs tag, frc::Pose2d currentPose);

    subsystems::CommandSwerveDrivetrain *m_chassis;
    frc::DriverStation::Alliance m_allianceColor;
    FieldConstants *m_fieldConstants;

    frc::Pose2d m_redReefCenter;
    frc::Pose2d m_blueReefCenter;

    ZoneParams *m_reefZonesRed;
    ZoneParams *m_reefZonesBlue;
};
