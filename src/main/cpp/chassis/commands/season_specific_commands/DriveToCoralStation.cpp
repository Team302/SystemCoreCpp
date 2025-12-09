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
#include "chassis/commands/season_specific_commands/DriveToCoralStation.h"
#include "fielddata/FieldConstants.h"
#include "fielddata/CoralStationHelper.h"
#include "state/RobotState.h"

DriveToCoralStation::DriveToCoralStation(subsystems::CommandSwerveDrivetrain *chassis)
    : DriveToPose(chassis)
{
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::DesiredCoralSide_Int);
}

frc::Pose2d DriveToCoralStation::GetEndPose()
{

    auto taginfo = CoralStationHelper::GetInstance()->GetNearestCoralStationTag();
    auto fieldconst = FieldConstants::GetInstance();
    frc::Pose2d endPose;

    if (taginfo.has_value())
    {
        auto tag = taginfo.value();
        auto tagpose{fieldconst->GetAprilTagPose2d(tag)};
        if (m_desiredCoralSide == RobotStateChanges::DesiredCoralSide::Sidewall)
        {
            auto sidewall = CoralStationHelper::GetInstance()->GetNearestSideWallCoralStation(tag);
            if (sidewall.has_value())
            {
                endPose = fieldconst->GetFieldElementPose2d(sidewall.value());
            }
        }
        else // CLOSEST_CORAL_STATION_ALLIANCE_SIDE
        {
            auto alliance = CoralStationHelper::GetInstance()->GetNearestAllianceWallCoralStation(tag);
            if (alliance.has_value())
            {
                endPose = fieldconst->GetFieldElementPose2d(alliance.value());
            }
        }
    }
    return endPose;
}

void DriveToCoralStation::NotifyStateUpdate(RobotStateChanges::StateChange change, int value)
{
    if (change == RobotStateChanges::StateChange::DesiredCoralSide_Int)
    {
        m_desiredCoralSide = RobotStateChanges::DesiredCoralSide(value); // cast to enum
    }
}