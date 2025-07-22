
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
#include "fielddata/ReefHelper.h"
#include "frc/DriverStation.h"
#include "utils/FMSData.h"
#include "utils/logging/debug/Logger.h"

ReefHelper *ReefHelper::m_instance = nullptr;
ReefHelper *ReefHelper::GetInstance()
{
    if (ReefHelper::m_instance == nullptr)
    {
        ReefHelper::m_instance = new ReefHelper();
    }
    return ReefHelper::m_instance;
}

ReefHelper::ReefHelper() : m_chassis(ChassisConfigMgr::GetInstance()->GetSwerveChassis()),
                           m_allianceColor(FMSData::GetAllianceColor()),
                           m_fieldConstants(FieldConstants::GetInstance())
{
    m_blueReefCenter = m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_REEF_CENTER);
    m_redReefCenter = m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_REEF_CENTER);
    InitZones();
}

std::optional<FieldConstants::AprilTagIDs> ReefHelper::GetNearestReefTag()
{
    m_allianceColor = FMSData::GetAllianceColor();
    if (m_chassis != nullptr)
    {
        auto pose = m_chassis->GetPose();
        units::length::meter_t distance = 0.0_m;
        auto chassisX = pose.X();
        if (m_allianceColor == frc::DriverStation::Alliance::kRed)
        {

            // Determine which side of the reef the robot is on by comparing X position to the x position of the center of the reef.
            // This will tell us which side of the reef can potentially have the closes face.  Then calculate the distance to the face
            // that is parallel to the alliance wall.  Compare that distance to one of the other sides of the reef that is on that half of
            // the reef.  If it is lower, then that face is the closest, if not compare to the other face on that half of the reef.  It this face
            // or the parallel face that will be the closest.
            //
            // So, in the picture below, first determine if it is either abc or def that are the potential closest faces.
            // Then, first calculate the distance of either b or e.  Now if you compare a or d and it is closer that b or e, then it is closer.
            // Otherwise, we need to compare c or f to see if they are closer.
            //
            //                                        *
            //                                   a  *   * d
            //                                     #     #
            //                                 b   |  +  | e
            //                                     #     #
            //                                  c   *   * f
            //                                        *
            //

            if (chassisX >= m_redReefCenter.X())
            {
                distance = CalcDistanceToAprilTag(FieldConstants::AprilTagIDs::RED_REEF_AB_TAG, pose);
                if (CalcDistanceToAprilTag(FieldConstants::AprilTagIDs::RED_REEF_CD_TAG, pose) < distance)
                {
                    return FieldConstants::AprilTagIDs::RED_REEF_CD_TAG;
                }
                else if (CalcDistanceToAprilTag(FieldConstants::AprilTagIDs::RED_REEF_KL_TAG, pose) < distance)
                {
                    return FieldConstants::AprilTagIDs::RED_REEF_KL_TAG;
                }
                return FieldConstants::AprilTagIDs::RED_REEF_AB_TAG;
            }
            else
            {
                distance = CalcDistanceToAprilTag(FieldConstants::AprilTagIDs::RED_REEF_GH_TAG, pose);
                if (CalcDistanceToAprilTag(FieldConstants::AprilTagIDs::RED_REEF_EF_TAG, pose) < distance)
                {
                    return FieldConstants::AprilTagIDs::RED_REEF_EF_TAG;
                }
                else if (CalcDistanceToAprilTag(FieldConstants::AprilTagIDs::RED_REEF_IJ_TAG, pose) < distance)
                {
                    return FieldConstants::AprilTagIDs::RED_REEF_IJ_TAG;
                }
                return FieldConstants::AprilTagIDs::RED_REEF_GH_TAG;
            }
        }
        else
        {
            if (chassisX <= m_blueReefCenter.X())
            {
                distance = CalcDistanceToAprilTag(FieldConstants::AprilTagIDs::BLUE_REEF_AB_TAG, pose);
                if (CalcDistanceToAprilTag(FieldConstants::AprilTagIDs::BLUE_REEF_CD_TAG, pose) < distance)
                {
                    return FieldConstants::AprilTagIDs::BLUE_REEF_CD_TAG;
                }
                else if (CalcDistanceToAprilTag(FieldConstants::AprilTagIDs::BLUE_REEF_KL_TAG, pose) < distance)
                {
                    return FieldConstants::AprilTagIDs::BLUE_REEF_KL_TAG;
                }
                return FieldConstants::AprilTagIDs::BLUE_REEF_AB_TAG;
            }
            else
            {
                distance = CalcDistanceToAprilTag(FieldConstants::AprilTagIDs::BLUE_REEF_GH_TAG, pose);
                if (CalcDistanceToAprilTag(FieldConstants::AprilTagIDs::BLUE_REEF_EF_TAG, pose) < distance)
                {
                    return FieldConstants::AprilTagIDs::BLUE_REEF_EF_TAG;
                }
                else if (CalcDistanceToAprilTag(FieldConstants::AprilTagIDs::BLUE_REEF_IJ_TAG, pose) < distance)
                {
                    return FieldConstants::AprilTagIDs::BLUE_REEF_IJ_TAG;
                }
                return FieldConstants::AprilTagIDs::BLUE_REEF_GH_TAG;
            }
        }
    }

    return std::nullopt;
}

std::optional<FieldConstants::FIELD_ELEMENT> ReefHelper::GetNearestLeftReefBranch(FieldConstants::AprilTagIDs tag)
{
    if (m_allianceColor == frc::DriverStation::Alliance::kRed)
    {
        switch (tag) // nearest april
        {
        case FieldConstants::AprilTagIDs::RED_REEF_AB_TAG:
            return FieldConstants::FIELD_ELEMENT::RED_REEF_A;
            break;
        case FieldConstants::AprilTagIDs::RED_REEF_CD_TAG:
            return FieldConstants::FIELD_ELEMENT::RED_REEF_C;
            break;
        case FieldConstants::AprilTagIDs::RED_REEF_EF_TAG:
            return FieldConstants::FIELD_ELEMENT::RED_REEF_E;
            break;
        case FieldConstants::AprilTagIDs::RED_REEF_GH_TAG:
            return FieldConstants::FIELD_ELEMENT::RED_REEF_G;
            break;
        case FieldConstants::AprilTagIDs::RED_REEF_IJ_TAG:
            return FieldConstants::FIELD_ELEMENT::RED_REEF_I;
            break;
        case FieldConstants::AprilTagIDs::RED_REEF_KL_TAG:
            return FieldConstants::FIELD_ELEMENT::RED_REEF_K;
            break;
        default:
            break;
        }
    }
    else
    {
        switch (tag) // nearest april
        {
        case FieldConstants::AprilTagIDs::BLUE_REEF_AB_TAG:
            return FieldConstants::FIELD_ELEMENT::BLUE_REEF_A;
            break;
        case FieldConstants::AprilTagIDs::BLUE_REEF_CD_TAG:
            return FieldConstants::FIELD_ELEMENT::BLUE_REEF_C;
            break;
        case FieldConstants::AprilTagIDs::BLUE_REEF_EF_TAG:
            return FieldConstants::FIELD_ELEMENT::BLUE_REEF_E;
            break;
        case FieldConstants::AprilTagIDs::BLUE_REEF_GH_TAG:
            return FieldConstants::FIELD_ELEMENT::BLUE_REEF_G;
            break;
        case FieldConstants::AprilTagIDs::BLUE_REEF_IJ_TAG:
            return FieldConstants::FIELD_ELEMENT::BLUE_REEF_I;
            break;
        case FieldConstants::AprilTagIDs::BLUE_REEF_KL_TAG:
            return FieldConstants::FIELD_ELEMENT::BLUE_REEF_K;
            break;
        default:
            break;
        }
    }
    return std::nullopt;
}

std::optional<FieldConstants::FIELD_ELEMENT> ReefHelper::GetNearestRightReefBranch(FieldConstants::AprilTagIDs tag)
{
    if (m_allianceColor == frc::DriverStation::Alliance::kRed)
    {

        switch (tag) // nearest april
        {
        case FieldConstants::AprilTagIDs::RED_REEF_AB_TAG:
            return FieldConstants::FIELD_ELEMENT::RED_REEF_B;
            break;
        case FieldConstants::AprilTagIDs::RED_REEF_CD_TAG:
            return FieldConstants::FIELD_ELEMENT::RED_REEF_D;
            break;
        case FieldConstants::AprilTagIDs::RED_REEF_EF_TAG:
            return FieldConstants::FIELD_ELEMENT::RED_REEF_F;
            break;
        case FieldConstants::AprilTagIDs::RED_REEF_GH_TAG:
            return FieldConstants::FIELD_ELEMENT::RED_REEF_H;
            break;
        case FieldConstants::AprilTagIDs::RED_REEF_IJ_TAG:
            return FieldConstants::FIELD_ELEMENT::RED_REEF_J;
            break;
        case FieldConstants::AprilTagIDs::RED_REEF_KL_TAG:
            return FieldConstants::FIELD_ELEMENT::RED_REEF_L;
            break;
        default:
            break;
        }
    }
    else
    {
        switch (tag) // nearest april
        {
        case FieldConstants::AprilTagIDs::BLUE_REEF_AB_TAG:
            return FieldConstants::FIELD_ELEMENT::BLUE_REEF_B;
            break;
        case FieldConstants::AprilTagIDs::BLUE_REEF_CD_TAG:
            return FieldConstants::FIELD_ELEMENT::BLUE_REEF_D;
            break;
        case FieldConstants::AprilTagIDs::BLUE_REEF_EF_TAG:
            return FieldConstants::FIELD_ELEMENT::BLUE_REEF_F;
            break;
        case FieldConstants::AprilTagIDs::BLUE_REEF_GH_TAG:
            return FieldConstants::FIELD_ELEMENT::BLUE_REEF_H;
            break;
        case FieldConstants::AprilTagIDs::BLUE_REEF_IJ_TAG:
            return FieldConstants::FIELD_ELEMENT::BLUE_REEF_J;
            break;
        case FieldConstants::AprilTagIDs::BLUE_REEF_KL_TAG:
            return FieldConstants::FIELD_ELEMENT::BLUE_REEF_L;
            break;
        default:
            break;
        }
    }
    return std::nullopt;
}

units::length::meter_t ReefHelper::CalcDistanceToAprilTag(FieldConstants::AprilTagIDs tag, frc::Pose2d currentPose)
{
    return currentPose.Translation().Distance(m_fieldConstants->GetAprilTagPose2d(tag).Translation());
}
void ReefHelper::InitZones()
{
    m_reefZonesRed = ZoneParser::ParseXML("RedReefPlacerZones.xml");
    m_reefZonesBlue = ZoneParser::ParseXML("BlueReefPlacerZones.xml");
}

void ReefHelper::IsInZone()
{
    m_allianceColor = FMSData::GetAllianceColor();
    auto reefZones = m_allianceColor == frc::DriverStation::Alliance::kRed ? m_reefZonesRed : m_reefZonesBlue;

    if (reefZones != nullptr)
    {
        bool intheZone = reefZones->IsPoseInZone(m_chassis->GetPose());

        RobotState::GetInstance()->PublishStateChange(RobotStateChanges::StateChange::IsInReefZone_Bool, intheZone);
    }
}