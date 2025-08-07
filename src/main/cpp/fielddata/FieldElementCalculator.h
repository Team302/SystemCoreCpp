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

// c++ includes
#include <map>

#include <fielddata/FieldConstants.h>
#include <frc/geometry/Pose3d.h>

struct TransformToPose
{
    FieldConstants::FIELD_ELEMENT referencePose;
    frc::Transform3d transform;
};
enum OffsetEnums
{
    // Comp Red
    COMP_RIGHT_RED,
    COMP_LEFT_RED,
    // Comp Blue
    COMP_RIGHT_BLUE,
    COMP_LEFT_BLUE,
    // Practice Red
    PRACTICE_LEFT_RED,
    PRACTICE_RIGHT_RED,
    // Practice Blue
    PRACTICE_LEFT_BLUE,
    PRACTICE_RIGHT_BLUE
};

class FieldElementCalculator
{

public:
    void CalcPositionsForField(robin_hood::unordered_map<FieldConstants::FIELD_ELEMENT, frc::Pose3d> &fieldConstantsPoseMap);
    frc::Pose3d CalcOffsetPositionForElement(frc::Pose3d &poseOfFaceTag, FieldConstants::FIELD_ELEMENT_OFFSETS offset);

private:
    void InitializeTransforms();
    void UpdateReefStickRobotTransforms();
    void InitializeReefBranchTransformsMap();
    void CalculateCenters(robin_hood::unordered_map<FieldConstants::FIELD_ELEMENT, frc::Pose3d> &fieldConstantsPoseMap);
    frc::Pose3d AverageHexagonPose(frc::Pose3d &pose1, frc::Pose3d &pose2, frc::Pose3d &pose3, frc::Pose3d &pose4, frc::Pose3d &pose5, frc::Pose3d &pose6);

    // Robot is 34" from front to back
    frc::Transform3d m_halfRobotTransform = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(16.35), // 16
            units::length::inch_t(0.0),
            units::length::inch_t(0.0)),
        frc::Rotation3d());

    robin_hood::unordered_map<OffsetEnums, frc::Transform3d> m_reefBranchOffsetMap;

    // other transforms
    frc::Transform3d m_noTransform = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(0.0),
            units::length::inch_t(0.0)),
        frc::Rotation3d());

    frc::Transform3d m_calcCoralLeftAlliance = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(-25.0),
            units::length::inch_t(0.0)),
        frc::Rotation3d());

    frc::Transform3d m_calcCoralLeftSidewall = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(30.0),
            units::length::inch_t(0.0)),
        frc::Rotation3d());

    frc::Transform3d m_calcCoralRightAlliance = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(30.0),
            units::length::inch_t(0.0)),
        frc::Rotation3d());

    frc::Transform3d m_calcCoralRightSidewall = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0),
            units::length::inch_t(-22.0),
            units::length::inch_t(0.0)),
        frc::Rotation3d());

    frc::Transform3d m_calcCageLeft = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(-18.0),
            units::length::inch_t(-43.0),
            units::length::inch_t(0.0)),
        frc::Rotation3d());

    frc::Transform3d m_calcCageRight = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(-18.0),
            units::length::inch_t(41.0),
            units::length::inch_t(0.0)),
        frc::Rotation3d());

    frc::Transform3d m_calcCageCenter = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(-18.0),
            units::length::inch_t(-1.0),
            units::length::inch_t(0.0)),
        frc::Rotation3d());

    frc::Transform3d m_calcLeftStick = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(-6.5),
            units::length::inch_t(0.0)),
        frc::Rotation3d());

    frc::Transform3d m_calcRightStick = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(0.0),
            units::length::inch_t(6.5),
            units::length::inch_t(0.0)),
        frc::Rotation3d());

    frc::Transform3d m_calcBargeFront = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(20.0), // 16.0
            units::length::inch_t(0.0),
            units::length::inch_t(0.0)),
        frc::Rotation3d());

    frc::Transform3d m_calcBargeBack = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(20.0), // 16.0
            units::length::inch_t(0.0),
            units::length::inch_t(0.0)),
        frc::Rotation3d());

    frc::Transform3d m_calcProcessorRed = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(16.0),
            units::length::inch_t(-9.0),
            units::length::inch_t(0.0)),
        frc::Rotation3d());
    frc::Transform3d m_calcProcessorBlue = frc::Transform3d(
        frc::Translation3d(
            units::length::inch_t(16.0),
            units::length::inch_t(-9.0),
            units::length::inch_t(0.0)),
        frc::Rotation3d());

    robin_hood::unordered_map<FieldConstants::FIELD_ELEMENT, TransformToPose> m_transformCalculatedMap;
    robin_hood::unordered_map<FieldConstants::FIELD_ELEMENT, TransformToPose> m_transformTagsMap;
};
