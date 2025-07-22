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
#include "FieldElementCalculator.h"

#include <frc/RobotController.h>

#include "FieldConstantsPoseLogger.h"
#include "RobotIdentifier.h"
#include "utils/FMSData.h"
#include "utils/logging/debug/Logger.h"
#include "vision/DragonVisionStructLogger.h"

void FieldElementCalculator::CalcPositionsForField(robin_hood::unordered_map<FieldConstants::FIELD_ELEMENT, frc::Pose3d> &fieldConstantsPoseMap)
{
    InitializeReefBranchTransformsMap();
    UpdateReefStickRobotTransforms();
    InitializeTransforms();
    CalculateCenters(fieldConstantsPoseMap);

    // update all of the calculated values only
    for (auto &[key, translatedPose] : m_transformCalculatedMap)
    {
        fieldConstantsPoseMap[key] = fieldConstantsPoseMap[m_transformCalculatedMap[key].referencePose] + m_transformCalculatedMap[key].transform + m_halfRobotTransform;
    }

    // after transform the tags, if the tags are transformed first it doubly transforms the calculated values
    for (auto &[key, unusedValue] : m_transformTagsMap)
    {
        fieldConstantsPoseMap[key] = fieldConstantsPoseMap[key] + m_halfRobotTransform;
    }

#ifdef INCLUDE_FIELD_ELEMENT_POSE_LOGGER
    FieldConstantsPoseLogger fpl;
    fpl.LogFieldElementPoses(fieldConstantsPoseMap);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("PoseLogger"), std::string("Logging"), std::string(""));
#endif
}

frc::Pose3d FieldElementCalculator::CalcOffsetPositionForElement(frc::Pose3d &poseOfFaceTag, FieldConstants::FIELD_ELEMENT_OFFSETS offset)
{

    frc::Transform3d transformToApply = m_calcLeftStick;
    if (offset == FieldConstants::FIELD_ELEMENT_OFFSETS::RIGHT_STICK)
    {
        transformToApply = m_calcRightStick;
    }
    return poseOfFaceTag + transformToApply + m_halfRobotTransform;
}

void FieldElementCalculator::UpdateReefStickRobotTransforms()
{
    int32_t teamNumber = frc::RobotController::GetTeamNumber();
    bool allianceIsBlue = FMSData::GetAllianceColor() == frc::DriverStation::Alliance::kBlue;

    if ((RobotIdentifier)teamNumber == RobotIdentifier::COMP_BOT_302)
    {
        m_calcLeftStick = allianceIsBlue ? m_calcLeftStick + m_reefBranchOffsetMap[OffsetEnums::COMP_LEFT_BLUE] : m_calcLeftStick + m_reefBranchOffsetMap[OffsetEnums::COMP_LEFT_RED];
        m_calcRightStick = allianceIsBlue ? m_calcRightStick + m_reefBranchOffsetMap[OffsetEnums::COMP_RIGHT_BLUE] : m_calcRightStick + m_reefBranchOffsetMap[OffsetEnums::COMP_RIGHT_RED];
    }
    else
    {
        m_calcLeftStick = allianceIsBlue ? m_calcLeftStick + m_reefBranchOffsetMap[PRACTICE_LEFT_BLUE] : m_calcLeftStick + m_reefBranchOffsetMap[PRACTICE_LEFT_RED];
        m_calcRightStick = allianceIsBlue ? m_calcRightStick + m_reefBranchOffsetMap[PRACTICE_RIGHT_BLUE] : m_calcRightStick + m_reefBranchOffsetMap[PRACTICE_RIGHT_RED];
    }
}
void FieldElementCalculator::InitializeReefBranchTransformsMap()
{
    // comp bot
    m_reefBranchOffsetMap[OffsetEnums::COMP_LEFT_BLUE] = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(m_reefBranchXOffset),
            units::length::inch_t(m_reefBranchYOffset),
            units::length::inch_t(m_reefBranchZOffset)),
        frc::Rotation3d());
    m_reefBranchOffsetMap[OffsetEnums::COMP_RIGHT_BLUE] = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(m_reefBranchXOffset),
            units::length::inch_t(m_reefBranchYOffset),
            units::length::inch_t(m_reefBranchZOffset)),
        frc::Rotation3d());
    m_reefBranchOffsetMap[OffsetEnums::COMP_LEFT_RED] = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(m_reefBranchXOffset),
            units::length::inch_t(m_reefBranchYOffset),
            units::length::inch_t(m_reefBranchZOffset)),
        frc::Rotation3d());
    m_reefBranchOffsetMap[OffsetEnums::COMP_RIGHT_RED] = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(m_reefBranchXOffset),
            units::length::inch_t(m_reefBranchYOffset),
            units::length::inch_t(m_reefBranchZOffset)),
        frc::Rotation3d());
    // practice bot
    m_reefBranchOffsetMap[OffsetEnums::PRACTICE_LEFT_BLUE] = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(m_reefBranchXOffset),
            units::length::inch_t(m_reefBranchYOffset),
            units::length::inch_t(m_reefBranchZOffset)),
        frc::Rotation3d());
    m_reefBranchOffsetMap[OffsetEnums::PRACTICE_RIGHT_BLUE] = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(m_reefBranchXOffset),
            units::length::inch_t(m_reefBranchYOffset),
            units::length::inch_t(m_reefBranchZOffset)),
        frc::Rotation3d());
    m_reefBranchOffsetMap[OffsetEnums::PRACTICE_LEFT_RED] = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(m_reefBranchXOffset),
            units::length::inch_t(m_reefBranchYOffset),
            units::length::inch_t(m_reefBranchZOffset)),
        frc::Rotation3d());
    m_reefBranchOffsetMap[OffsetEnums::PRACTICE_RIGHT_RED] = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(m_reefBranchXOffset),
            units::length::inch_t(m_reefBranchYOffset),
            units::length::inch_t(m_reefBranchZOffset)),
        frc::Rotation3d());
}

void FieldElementCalculator::InitializeTransforms()
{

    // no transforms for april tags on blue side
    m_transformTagsMap[FieldConstants::FIELD_ELEMENT::BLUE_CORAL_STATION_LEFT];
    m_transformTagsMap[FieldConstants::FIELD_ELEMENT::BLUE_CORAL_STATION_RIGHT];
    m_transformTagsMap[FieldConstants::FIELD_ELEMENT::BLUE_PROCESSOR];
    m_transformTagsMap[FieldConstants::FIELD_ELEMENT::BLUE_BARGE_FRONT];
    m_transformTagsMap[FieldConstants::FIELD_ELEMENT::BLUE_BARGE_BACK];
    m_transformTagsMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_AB];
    m_transformTagsMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_CD];
    m_transformTagsMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_EF];
    m_transformTagsMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_GH];
    m_transformTagsMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_IJ];
    m_transformTagsMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_KL];

    // Blue Calculated Positions
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_CORAL_STATION_LEFT_ALLIANCE] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_CORAL_STATION_LEFT, m_calcCoralLeftAlliance);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_CORAL_STATION_LEFT_SIDEWALL] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_CORAL_STATION_LEFT, m_calcCoralLeftSidewall);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_CORAL_STATION_RIGHT_ALLIANCE] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_CORAL_STATION_RIGHT, m_calcCoralRightAlliance);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_CORAL_STATION_RIGHT_SIDEWALL] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_CORAL_STATION_RIGHT, m_calcCoralRightSidewall);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_LEFT_CAGE] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_BARGE_FRONT, m_calcCageLeft);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_RIGHT_CAGE] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_BARGE_FRONT, m_calcCageRight);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_CENTER_CAGE] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_BARGE_FRONT, m_calcCageCenter);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_BARGE_FRONT_CALCULATED] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_BARGE_FRONT, m_calcBargeFront);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_BARGE_BACK_CALCULATED] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_BARGE_BACK, m_calcBargeBack);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_A] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_REEF_AB, m_calcLeftStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_B] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_REEF_AB, m_calcRightStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_C] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_REEF_CD, m_calcLeftStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_D] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_REEF_CD, m_calcRightStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_E] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_REEF_EF, m_calcLeftStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_F] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_REEF_EF, m_calcRightStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_G] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_REEF_GH, m_calcLeftStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_H] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_REEF_GH, m_calcRightStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_I] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_REEF_IJ, m_calcLeftStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_J] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_REEF_IJ, m_calcRightStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_K] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_REEF_KL, m_calcLeftStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_L] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_REEF_KL, m_calcRightStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::BLUE_PROCESSOR_CALCULATED] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::BLUE_PROCESSOR, m_calcProcessorBlue);

    // no transforms for april tags on red side
    m_transformTagsMap[FieldConstants::FIELD_ELEMENT::RED_CORAL_STATION_LEFT];
    m_transformTagsMap[FieldConstants::FIELD_ELEMENT::RED_CORAL_STATION_RIGHT];
    m_transformTagsMap[FieldConstants::FIELD_ELEMENT::RED_BARGE_FRONT];
    m_transformTagsMap[FieldConstants::FIELD_ELEMENT::RED_BARGE_BACK];
    m_transformTagsMap[FieldConstants::FIELD_ELEMENT::RED_REEF_AB];
    m_transformTagsMap[FieldConstants::FIELD_ELEMENT::RED_REEF_CD];
    m_transformTagsMap[FieldConstants::FIELD_ELEMENT::RED_REEF_EF];
    m_transformTagsMap[FieldConstants::FIELD_ELEMENT::RED_REEF_GH];
    m_transformTagsMap[FieldConstants::FIELD_ELEMENT::RED_REEF_IJ];
    m_transformTagsMap[FieldConstants::FIELD_ELEMENT::RED_REEF_KL];

    // Red Calculated Positions
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_CORAL_STATION_LEFT_ALLIANCE] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_CORAL_STATION_LEFT, m_calcCoralLeftAlliance);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_CORAL_STATION_LEFT_SIDEWALL] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_CORAL_STATION_LEFT, m_calcCoralLeftSidewall);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_CORAL_STATION_RIGHT_ALLIANCE] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_CORAL_STATION_RIGHT, m_calcCoralRightAlliance);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_CORAL_STATION_RIGHT_SIDEWALL] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_CORAL_STATION_RIGHT, m_calcCoralRightSidewall);

    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_LEFT_CAGE] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_BARGE_FRONT, m_calcCageLeft);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_RIGHT_CAGE] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_BARGE_FRONT, m_calcCageRight);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_CENTER_CAGE] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_BARGE_FRONT, m_calcCageCenter);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_BARGE_FRONT_CALCULATED] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_BARGE_FRONT, m_calcBargeFront);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_BARGE_BACK_CALCULATED] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_BARGE_BACK, m_calcBargeBack);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_REEF_A] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_REEF_AB, m_calcLeftStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_REEF_B] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_REEF_AB, m_calcRightStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_REEF_C] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_REEF_CD, m_calcLeftStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_REEF_D] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_REEF_CD, m_calcRightStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_REEF_E] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_REEF_EF, m_calcLeftStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_REEF_F] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_REEF_EF, m_calcRightStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_REEF_G] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_REEF_GH, m_calcLeftStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_REEF_H] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_REEF_GH, m_calcRightStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_REEF_I] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_REEF_IJ, m_calcLeftStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_REEF_J] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_REEF_IJ, m_calcRightStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_REEF_K] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_REEF_KL, m_calcLeftStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_REEF_L] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_REEF_KL, m_calcRightStick);
    m_transformCalculatedMap[FieldConstants::FIELD_ELEMENT::RED_PROCESSOR_CALCULATED] =
        TransformToPose(FieldConstants::FIELD_ELEMENT::RED_PROCESSOR, m_calcProcessorRed);
}

void FieldElementCalculator::CalculateCenters(robin_hood::unordered_map<FieldConstants::FIELD_ELEMENT, frc::Pose3d> &fieldConstantsPoseMap)
{
    fieldConstantsPoseMap[FieldConstants::FIELD_ELEMENT::RED_REEF_CENTER] = AverageHexagonPose(
        fieldConstantsPoseMap[FieldConstants::FIELD_ELEMENT::RED_REEF_AB],
        fieldConstantsPoseMap[FieldConstants::FIELD_ELEMENT::RED_REEF_CD],
        fieldConstantsPoseMap[FieldConstants::FIELD_ELEMENT::RED_REEF_EF],
        fieldConstantsPoseMap[FieldConstants::FIELD_ELEMENT::RED_REEF_GH],
        fieldConstantsPoseMap[FieldConstants::FIELD_ELEMENT::RED_REEF_IJ],
        fieldConstantsPoseMap[FieldConstants::FIELD_ELEMENT::RED_REEF_KL]);

    fieldConstantsPoseMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_CENTER] = AverageHexagonPose(
        fieldConstantsPoseMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_AB],
        fieldConstantsPoseMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_CD],
        fieldConstantsPoseMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_EF],
        fieldConstantsPoseMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_GH],
        fieldConstantsPoseMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_IJ],
        fieldConstantsPoseMap[FieldConstants::FIELD_ELEMENT::BLUE_REEF_KL]);
}

frc::Pose3d FieldElementCalculator::AverageHexagonPose(frc::Pose3d &pose1, frc::Pose3d &pose2, frc::Pose3d &pose3, frc::Pose3d &pose4, frc::Pose3d &pose5, frc::Pose3d &pose6)
{

    units::length::meter_t averageX = (pose1.X() + pose2.X() + pose3.X() + pose4.X() + pose5.X() + pose6.X()) / 6;
    units::length::meter_t averageY = (pose1.Y() + pose2.Y() + pose3.Y() + pose4.Y() + pose5.Y() + pose6.Y()) / 6;
    units::length::meter_t averageZ = (pose1.Z() + pose2.Z() + pose3.Z() + pose4.Z() + pose5.Z() + pose6.Z()) / 6;

    return frc::Pose3d(averageX, averageY, averageZ, frc::Rotation3d());
}