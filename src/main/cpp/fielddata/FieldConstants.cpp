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
#include "fielddata/FieldConstants.h"

#include "fielddata/FieldAprilTagIDs.h"
#include "fielddata/FieldElementCalculator.h"

FieldConstants *FieldConstants::m_instance = nullptr;
FieldConstants *FieldConstants::GetInstance()
{
    if (FieldConstants::m_instance == nullptr)
    {
        FieldConstants::m_instance = new FieldConstants();
    }
    return FieldConstants::m_instance;
}

FieldConstants::FieldConstants()
{
    ReadFieldCalibrationData();

    // Blue AprilTag locations
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_CORAL_STATION_LEFT] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::BLUE_CORAL_STATION_LEFT_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_CORAL_STATION_RIGHT] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::BLUE_CORAL_STATION_RIGHT_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_PROCESSOR] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::BLUE_PROCESSOR_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_BARGE_FRONT] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::BLUE_BARGE_FRONT_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_BARGE_BACK] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::BLUE_BARGE_BACK_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_AB] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::BLUE_REEF_AB_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_CD] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::BLUE_REEF_CD_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_EF] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::BLUE_REEF_EF_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_GH] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::BLUE_REEF_GH_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_IJ] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::BLUE_REEF_IJ_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_KL] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::BLUE_REEF_KL_TAG));

    // Blue Calculated Positions
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_CORAL_STATION_LEFT_ALLIANCE] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_CORAL_STATION_LEFT_SIDEWALL] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_CORAL_STATION_RIGHT_ALLIANCE] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_CORAL_STATION_RIGHT_SIDEWALL] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_RIGHT_CAGE] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_LEFT_CAGE] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_CENTER] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_A] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_B] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_C] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_D] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_E] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_F] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_G] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_H] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_I] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_J] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_K] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_REEF_L] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_BARGE_FRONT_CALCULATED] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_BARGE_BACK_CALCULATED] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::BLUE_PROCESSOR_CALCULATED] = m_placeholder;

    // Red AprilTag locations
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_CORAL_STATION_LEFT] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::RED_CORAL_STATION_LEFT_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_CORAL_STATION_RIGHT] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::RED_CORAL_STATION_RIGHT_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_PROCESSOR] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::RED_PROCESSOR_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_BARGE_FRONT] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::RED_BARGE_FRONT_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_BARGE_BACK] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::RED_BARGE_BACK_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_AB] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::RED_REEF_AB_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_CD] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::RED_REEF_CD_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_EF] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::RED_REEF_EF_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_GH] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::RED_REEF_GH_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_IJ] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::RED_REEF_IJ_TAG));
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_KL] = GetAprilTagPoseFromLayout(static_cast<int>(FieldAprilTagIDs::RED_REEF_KL_TAG));

    // Red Calculated Positions
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_CORAL_STATION_LEFT_ALLIANCE] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_CORAL_STATION_LEFT_SIDEWALL] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_CORAL_STATION_RIGHT_ALLIANCE] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_CORAL_STATION_RIGHT_SIDEWALL] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_RIGHT_CAGE] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_LEFT_CAGE] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_CENTER] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_A] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_B] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_C] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_D] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_E] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_F] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_G] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_H] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_I] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_J] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_K] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_REEF_L] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_BARGE_FRONT_CALCULATED] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_BARGE_BACK_CALCULATED] = m_placeholder;
    fieldConstantsPoseMap[FIELD_ELEMENT::RED_PROCESSOR_CALCULATED] = m_placeholder;

    m_aprilTagPoseMap[1] = GetAprilTagPoseFromLayout(1);
    m_aprilTagPoseMap[2] = GetAprilTagPoseFromLayout(2);
    m_aprilTagPoseMap[3] = GetAprilTagPoseFromLayout(3);
    m_aprilTagPoseMap[4] = GetAprilTagPoseFromLayout(4);
    m_aprilTagPoseMap[5] = GetAprilTagPoseFromLayout(5);
    m_aprilTagPoseMap[6] = GetAprilTagPoseFromLayout(6);
    m_aprilTagPoseMap[7] = GetAprilTagPoseFromLayout(7);
    m_aprilTagPoseMap[8] = GetAprilTagPoseFromLayout(8);
    m_aprilTagPoseMap[9] = GetAprilTagPoseFromLayout(9);
    m_aprilTagPoseMap[10] = GetAprilTagPoseFromLayout(10);
    m_aprilTagPoseMap[11] = GetAprilTagPoseFromLayout(11);
    m_aprilTagPoseMap[12] = GetAprilTagPoseFromLayout(12);
    m_aprilTagPoseMap[13] = GetAprilTagPoseFromLayout(13);
    m_aprilTagPoseMap[14] = GetAprilTagPoseFromLayout(14);
    m_aprilTagPoseMap[15] = GetAprilTagPoseFromLayout(15);
    m_aprilTagPoseMap[16] = GetAprilTagPoseFromLayout(16);
    m_aprilTagPoseMap[17] = GetAprilTagPoseFromLayout(17);
    m_aprilTagPoseMap[18] = GetAprilTagPoseFromLayout(18);
    m_aprilTagPoseMap[19] = GetAprilTagPoseFromLayout(19);
    m_aprilTagPoseMap[20] = GetAprilTagPoseFromLayout(20);
    m_aprilTagPoseMap[21] = GetAprilTagPoseFromLayout(21);
    m_aprilTagPoseMap[22] = GetAprilTagPoseFromLayout(22);

    for (const auto &pair : m_aprilTagPoseMap)
    {
        m_aprilTag2dPoses[pair.first] = pair.second.ToPose2d();
    }

    FieldElementCalculator fc;
    fc.CalcPositionsForField(fieldConstantsPoseMap);

    for (const auto &pair : fieldConstantsPoseMap)
    {
        m_fieldConst2dPoses[static_cast<unsigned int>(pair.first)] = pair.second.ToPose2d();
    }
}
frc::Pose3d FieldConstants::GetFieldElementPose(FIELD_ELEMENT element)
{
    return fieldConstantsPoseMap[element];
}

frc::Pose2d FieldConstants::GetFieldElementPose2d(FIELD_ELEMENT element)
{
    return m_fieldConst2dPoses[static_cast<unsigned int>(element)];
}

void FieldConstants::ReadFieldCalibrationData()
{
    if (std::filesystem::exists(m_fieldFilePath))
    {
        m_aprilTagVector = frc::AprilTagFieldLayout(m_fieldFilePath).GetTags();
        for (unsigned int i = 1; i >= m_aprilTagVector.size(); i++)
        {
            frc::AprilTag tag = m_aprilTagVector[i];
            m_aprilTagPoseMap[i] = tag.pose;
        }
    }
}

frc::Pose3d FieldConstants::GetAprilTagPoseFromLayout(int tagID)
{
    if (!m_fieldLayout.GetTagPose(tagID).has_value())
    {
        return frc::Pose3d();
    }
    return m_fieldLayout.GetTagPose(tagID).value();
}

frc::Pose3d FieldConstants::GetAprilTagPose(FieldAprilTagIDs tag)
{
    return m_aprilTagPoseMap[static_cast<unsigned int>(tag)];
}

frc::Pose2d FieldConstants::GetAprilTagPose2d(FieldAprilTagIDs tag)
{
    return m_aprilTag2dPoses[static_cast<unsigned int>(tag)];
}
