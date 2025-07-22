
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

#include "chassis/ChassisConfigMgr.h"
#include "fielddata/CoralStationHelper.h"
#include "frc/DriverStation.h"
#include "utils/FMSData.h"

CoralStationHelper *CoralStationHelper::m_instance = nullptr;
CoralStationHelper *CoralStationHelper::GetInstance()
{
    if (CoralStationHelper::m_instance == nullptr)
    {
        CoralStationHelper::m_instance = new CoralStationHelper();
    }
    return CoralStationHelper::m_instance;
}

CoralStationHelper::CoralStationHelper() : m_chassis(ChassisConfigMgr::GetInstance()->GetSwerveChassis()),
                                           m_allianceColor(FMSData::GetAllianceColor()),
                                           m_fieldConstants(FieldConstants::GetInstance())
{
}

std::optional<FieldConstants::AprilTagIDs> CoralStationHelper::GetNearestCoralStationTag()
{
    if (m_chassis != nullptr)
    {

        auto pose = m_chassis->GetPose();
        ;
        if (m_allianceColor == frc::DriverStation::Alliance::kRed)
        {
            if (CalcDistanceToAprilTag(FieldConstants::AprilTagIDs::RED_CORAL_STATION_LEFT_TAG, pose) <
                CalcDistanceToAprilTag(FieldConstants::AprilTagIDs::RED_CORAL_STATION_RIGHT_TAG, pose))
            {
                return FieldConstants::AprilTagIDs::RED_CORAL_STATION_LEFT_TAG;
            }
            return FieldConstants::AprilTagIDs::RED_CORAL_STATION_RIGHT_TAG;
        }
        else
        {
            if (CalcDistanceToAprilTag(FieldConstants::AprilTagIDs::BLUE_CORAL_STATION_LEFT_TAG, pose) <
                CalcDistanceToAprilTag(FieldConstants::AprilTagIDs::BLUE_CORAL_STATION_RIGHT_TAG, pose))
            {
                return FieldConstants::AprilTagIDs::BLUE_CORAL_STATION_LEFT_TAG;
            }
            return FieldConstants::AprilTagIDs::BLUE_CORAL_STATION_RIGHT_TAG;
        }
    }
    return std::nullopt;
}

std::optional<FieldConstants::FIELD_ELEMENT> CoralStationHelper::GetNearestAllianceWallCoralStation(FieldConstants::AprilTagIDs tag)
{
    if (m_allianceColor == frc::DriverStation::Alliance::kRed)
    {
        switch (tag) // nearest april
        {
        case FieldConstants::AprilTagIDs::RED_CORAL_STATION_LEFT_TAG:
            return FieldConstants::FIELD_ELEMENT::RED_CORAL_STATION_LEFT_ALLIANCE;
            break;
        case FieldConstants::AprilTagIDs::RED_CORAL_STATION_RIGHT_TAG:
            return FieldConstants::FIELD_ELEMENT::RED_CORAL_STATION_RIGHT_ALLIANCE;
            break;
        default:
            break;
        }
    }
    else
    {
        switch (tag) // nearest april
        {
        case FieldConstants::AprilTagIDs::BLUE_CORAL_STATION_LEFT_TAG:
            return FieldConstants::FIELD_ELEMENT::BLUE_CORAL_STATION_LEFT_ALLIANCE;
            break;
        case FieldConstants::AprilTagIDs::BLUE_CORAL_STATION_RIGHT_TAG:
            return FieldConstants::FIELD_ELEMENT::BLUE_CORAL_STATION_RIGHT_ALLIANCE;
            break;
        default:
            break;
        }
    }
    return std::nullopt;
}

std::optional<FieldConstants::FIELD_ELEMENT> CoralStationHelper::GetNearestSideWallCoralStation(FieldConstants::AprilTagIDs tag)
{
    if (m_allianceColor == frc::DriverStation::Alliance::kRed)
    {
        switch (tag) // nearest april
        {
        case FieldConstants::AprilTagIDs::RED_CORAL_STATION_LEFT_TAG:
            return FieldConstants::FIELD_ELEMENT::RED_CORAL_STATION_LEFT_SIDEWALL;
            break;
        case FieldConstants::AprilTagIDs::RED_CORAL_STATION_RIGHT_TAG:
            return FieldConstants::FIELD_ELEMENT::RED_CORAL_STATION_RIGHT_SIDEWALL;
            break;
        default:
            break;
        }
    }
    else
    {
        switch (tag) // nearest april
        {
        case FieldConstants::AprilTagIDs::BLUE_CORAL_STATION_LEFT_TAG:
            return FieldConstants::FIELD_ELEMENT::BLUE_CORAL_STATION_LEFT_SIDEWALL;
            break;
        case FieldConstants::AprilTagIDs::BLUE_CORAL_STATION_RIGHT_TAG:
            return FieldConstants::FIELD_ELEMENT::BLUE_CORAL_STATION_RIGHT_SIDEWALL;
            break;
        default:
            break;
        }
    }
    return std::nullopt;
}

units::length::meter_t CoralStationHelper::CalcDistanceToAprilTag(FieldConstants::AprilTagIDs tag, frc::Pose2d currentPose)
{
    return currentPose.Translation().Distance(m_fieldConstants->GetAprilTagPose2d(tag).Translation());
}