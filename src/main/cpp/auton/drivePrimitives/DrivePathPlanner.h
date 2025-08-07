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
#include <tuple>

// Team302 Includes
#include "auton/PrimitiveParams.h"
#include "auton/drivePrimitives/IPrimitive.h"
#include "chassis/ChassisOptionEnums.h"
#include "chassis/SwerveChassis.h"
#include "chassis/states/TrajectoryDrivePathPlanner.h"
#include "utils/logging/signals/DragonDataLogger.h"
#include "fielddata/DragonTargetFinder.h"
#include "chassis/states/DriveToFieldElement.h"

// FRC,WPI Includes
#include "frc/geometry/Pose2d.h"
#include "frc/Timer.h"
#include "units/length.h"
#include "units/time.h"

// third party includes
#include "pathplanner/lib/trajectory/PathPlannerTrajectory.h"

class DrivePathPlanner : public IPrimitive, public DragonDataLogger
{
public:
    DrivePathPlanner();
    ~DrivePathPlanner() = default;

    void Init(PrimitiveParams *params) override;
    void Run() override;
    bool IsDone() override;
    void DataLog(uint64_t timestamp) override;

    int FindDriveToZoneIndex(ZoneParamsVector zones);

private:
    void InitMoveInfo();
    DriveToFieldElement *GetDriveToObject(ChassisOptionEnums::DriveStateType driveToType);
    bool IsInZone();

    void CheckForDriveTo();
    void LogMoveInfo();
    SwerveChassis *m_chassis;

    int m_currentPrim = 0;

    TrajectoryDrivePathPlanner *m_trajectoryDrivePathPlanner;
    std::unique_ptr<frc::Timer> m_timer;
    pathplanner::PathPlannerTrajectory m_trajectory;
    std::string m_pathname;
    std::string m_choreoTrajectoryName;
    ChassisOptionEnums::PathGainsType m_pathGainsType;
    units::time::second_t m_maxTime;
    std::string m_ntName;
    bool m_isVisionDrive;
    PrimitiveParams::VISION_ALIGNMENT m_visionAlignment;
    ChassisMovement m_moveInfo;
    units::length::meter_t m_centerLine = units::length::meter_t(8.27);
    units::length::meter_t m_offset = units::length::meter_t(1.0);
    units::length::meter_t m_chassisOffset = units::length::meter_t(0.5);

    bool m_checkForDriveToUpdate = false;
    bool m_updateTimeLatch = false;
    // const double m_percentageCompleteThreshold = 0.75;
    const units::length::meter_t m_distanceThreshold = units::length::meter_t(1.0);
    units::time::second_t m_totalTrajectoryTime;
    frc::Pose2d m_finalPose;

    DriveToFieldElement *m_driveToObject;

    std::tuple<TrajectoryDrivePathPlanner *, ChassisOptionEnums::DriveStateType> m_driveToInfo;

    ZoneParams *m_zone;
};
