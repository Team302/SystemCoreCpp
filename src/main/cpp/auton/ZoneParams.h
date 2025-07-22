
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

// C++ Includes
#include <vector>

// FRC includes

// Team 302 includes
#include "auton/AutonGrid.h"
#include "chassis/ChassisOptionEnums.h"
#include "mechanisms/DragonTale/DragonTale.h"
#include "auton/PrimitiveEnums.h"

// Third Party Includes

enum class ZoneMode
{
    NOTHING = -1,
    RECTANGLE,
    CIRCLE,
};

class ZoneParams
{
public:
    ZoneParams(frc::Pose2d circlePose,
               units::length::inch_t radius,
               units::length::meter_t m_xgrid1rect,
               units::length::meter_t m_xgrid2rect,
               units::length::meter_t m_ygrid1rect,
               units::length::meter_t m_ygrid2rect,
               bool isTaleStateChanging,
               DragonTale::STATE_NAMES taleOption,
               ChassisOptionEnums::AutonChassisOptions autonchassisoption,
               ChassisOptionEnums::HeadingOption headingOption,
               ChassisOptionEnums::DriveStateType pathUpdateOption,
               ChassisOptionEnums::AutonAvoidOptions autonavoidoption,
               ZoneMode zoneMode); // declare ZoneParams public constructor with parameters xgrid1, etc.

    ZoneParams() = delete;
    ~ZoneParams() = default; // Destructor

    units::length::meter_t GetX1Rect() const { return m_xgrid1rect; }
    units::length::meter_t GetX2Rect() const { return m_xgrid2rect; }
    units::length::meter_t GetY1Rect() const { return m_ygrid1rect; }
    units::length::meter_t GetY2Rect() const { return m_ygrid2rect; }

    ZoneMode GetZoneMode() const { return m_zoneMode; }

    frc::Pose2d GetCircleZonePose() const { return m_circlePose; }
    units::length::inch_t GetRadius() const { return m_radius; }

    ChassisOptionEnums::DriveStateType GetPathUpdateOption() const { return m_pathUpdateOption; }

    bool IsTaleStateChanging() const { return m_isTaleStateChanging; }

    DragonTale::STATE_NAMES GetTaleOption() const { return m_taleOption; }

    ChassisOptionEnums::HeadingOption GetHeadingOption() const { return m_headingOption; }
    ChassisOptionEnums::AutonChassisOptions GetChassisOption() const { return m_chassisoption; }
    ChassisOptionEnums::AutonAvoidOptions GetAvoidOption() const { return m_avoidoption; }

    bool IsPoseInZone(frc::Pose2d robotPose);

private:
    units::length::meter_t m_xgrid1rect;
    units::length::meter_t m_xgrid2rect;
    units::length::meter_t m_ygrid1rect;
    units::length::meter_t m_ygrid2rect;
    bool m_isTaleStateChanging;

    DragonTale::STATE_NAMES m_taleOption;

    ChassisOptionEnums::AutonChassisOptions m_chassisoption;
    ChassisOptionEnums::HeadingOption m_headingOption;
    ChassisOptionEnums::AutonAvoidOptions m_avoidoption; // instances of said parameters

    ChassisOptionEnums::DriveStateType m_pathUpdateOption;

    ZoneMode m_zoneMode;

    frc::Pose2d m_circlePose;
    units::length::inch_t m_radius;
};

typedef std::vector<ZoneParams *> ZoneParamsVector; // create typedef ZoneParamsVector
