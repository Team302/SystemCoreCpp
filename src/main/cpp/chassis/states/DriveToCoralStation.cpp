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
//=====================================================================================================================================================

// C++ Includes
#include <string>

// FRC Includes

// Team302 Includes
#include "chassis/states/DriveToCoralStation.h"
#include "fielddata/DragonTargetFinder.h"
#include "state/RobotState.h"
#include <frc/smartdashboard/SmartDashboard.h>

using std::string;

DriveToCoralStation::DriveToCoralStation(RobotDrive *robotDrive) : DriveToFieldElement(robotDrive)
{
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::GameState_Int);
    frc::SmartDashboard::PutBoolean(m_magicButtonKey, false);
}

string DriveToCoralStation::GetDriveStateName() const
{
    return std::string("DriveToCoralStation");
}

DragonTargetFinderTarget DriveToCoralStation::GetDriveToTarget() const
{
    if (m_gamePeroid == RobotStateChanges::Teleop && !m_runOnceLatch)
    {
        m_target = frc::SmartDashboard::GetBoolean(m_magicButtonKey, false) ? DragonTargetFinderTarget::CLOSEST_CORAL_STATION_ALLIANCE_SIDE : m_target;
        m_runOnceLatch = true;
    }
    return m_target;
}
ChassisOptionEnums::DriveStateType DriveToCoralStation::GetDriveStateType() const
{
    return ChassisOptionEnums::DriveStateType::DRIVE_TO_CORAL_STATION;
}
ChassisOptionEnums::HeadingOption DriveToCoralStation::GetHeadingOption() const
{
    return ChassisOptionEnums::HeadingOption::FACE_CORAL_STATION;
}
void DriveToCoralStation::NotifyStateUpdate(RobotStateChanges::StateChange change, int value)
{
    if (change == RobotStateChanges::GameState_Int)
    {
        m_gamePeroid = static_cast<RobotStateChanges::GamePeriod>(value);
    }
}
