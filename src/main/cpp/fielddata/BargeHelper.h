
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

// C++ includes
#include <optional>
#include <vector>
// 302 includes

#include "fielddata/FieldConstants.h"
#include "frc/DriverStation.h"
#include "frc/geometry/Pose2d.h"
#include "state/RobotState.h"
#include "auton/AutonGrid.h"
#include "auton/ZoneParser.h"
#include "auton/ZoneParams.h"
#include "fielddata/DragonTargetFinder.h"

enum class BargeZones
{
    FAR_LEFT_ZONE,
    CLOSE_LEFT_ZONE,
    CLOSE_RIGHT_ZONE,
    FAR_RIGHT_ZONE,
    NO_ZONE
};
class BargeHelper
{
public:
    static BargeHelper *GetInstance();
    void IsInZone();
    void InitZones();
    std::optional<units::length::meter_t> ClampChassisY();
    frc::Pose2d GetCagePose(DragonTargetFinderTarget target);

    frc::Pose2d CalcBargePose();

private:
    BargeHelper();
    ~BargeHelper() = default;
    static BargeHelper *m_instance;

    std::optional<BargeZones> GetClosestZone();
    void CalculateZones();

    subsystems::CommandSwerveDrivetrain *m_chassis;
    FieldConstants *m_fieldConstants;

    // blue
    const frc::Pose2d m_blueCenterOfBarge{8.225_m, 6.161_m, 0_deg};
    const frc::Pose2d m_blueLeftBargePose{8.225_m, 8.033_m, 0_deg};
    const frc::Pose2d m_blueRightBargePose{8.225_m, 4.329_m, 0_deg};

    // red
    const frc::Pose2d m_redLeftBargePose{9.358_m, 0.5_m, 0_deg};
    const frc::Pose2d m_redCenterOfBarge{9.358_m, 1.904_m, 0_deg};
    const frc::Pose2d m_redRightBargePose{9.358_m, 3.723_m, 0_deg};

    const unsigned int m_numOfZones = 4;
    std::vector<frc::Pose2d> m_zonesVector;
    ZoneParams *m_bargeZonesBlue;
    ZoneParams *m_bargeZonesRed;
    units::length::foot_t m_centerLine{28.73};
    units::length::meter_t m_redYClampMin{0.5_m};
    units::length::meter_t m_redYClampMax{3.0_m};
    units::length::meter_t m_blueYClampMin{5.0_m};
    units::length::meter_t m_blueYClampMax{7.5_m};

    frc::Rotation2d m_redBackBargeRotation{-170_deg};
    frc::Rotation2d m_blueBackBargeRotation{170_deg};
    frc::Rotation2d m_redFrontBargeRotation{-10_deg};
    frc::Rotation2d m_blueFrontBargeRotation{10_deg};
};
