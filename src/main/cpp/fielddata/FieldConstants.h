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
#include <filesystem>
#include <iostream>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>

#include "RobinHood/robin_hood.h"

#include "fielddata/FieldAprilTagIDs.h"
#include "frc/apriltag/AprilTagFieldLayout.h"

#include "units/angle.h"
#include "units/base.h"

class FieldConstants
{
public:
    static FieldConstants *GetInstance();
    enum class FIELD_ELEMENT
    {
        // 2025 - BLUE APRIL TAGS
        BLUE_CORAL_STATION_LEFT,
        BLUE_CORAL_STATION_RIGHT,
        BLUE_PROCESSOR,
        BLUE_BARGE_FRONT,
        BLUE_BARGE_BACK,
        BLUE_REEF_AB,
        BLUE_REEF_CD,
        BLUE_REEF_EF,
        BLUE_REEF_GH,
        BLUE_REEF_IJ,
        BLUE_REEF_KL,
        // 2025 - RED APRIL TAGS
        RED_CORAL_STATION_LEFT,
        RED_CORAL_STATION_RIGHT,
        RED_PROCESSOR,
        RED_BARGE_FRONT,
        RED_BARGE_BACK,
        RED_REEF_AB,
        RED_REEF_CD,
        RED_REEF_EF,
        RED_REEF_GH,
        RED_REEF_IJ,
        RED_REEF_KL,
        // 2025 - Blue Calculated Positions
        BLUE_CORAL_STATION_LEFT_ALLIANCE,
        BLUE_CORAL_STATION_LEFT_SIDEWALL,
        BLUE_CORAL_STATION_RIGHT_ALLIANCE,
        BLUE_CORAL_STATION_RIGHT_SIDEWALL,
        BLUE_LEFT_CAGE,
        BLUE_RIGHT_CAGE,
        BLUE_CENTER_CAGE,
        BLUE_REEF_CENTER,
        BLUE_REEF_A,
        BLUE_REEF_B,
        BLUE_REEF_C,
        BLUE_REEF_D,
        BLUE_REEF_E,
        BLUE_REEF_F,
        BLUE_REEF_G,
        BLUE_REEF_H,
        BLUE_REEF_I,
        BLUE_REEF_J,
        BLUE_REEF_K,
        BLUE_REEF_L,
        BLUE_BARGE_FRONT_CALCULATED,
        BLUE_BARGE_BACK_CALCULATED,
        BLUE_PROCESSOR_CALCULATED,
        // 2025 - Red Calculated Positions
        RED_CORAL_STATION_LEFT_ALLIANCE,
        RED_CORAL_STATION_LEFT_SIDEWALL,
        RED_CORAL_STATION_RIGHT_ALLIANCE,
        RED_CORAL_STATION_RIGHT_SIDEWALL,
        RED_LEFT_CAGE,
        RED_RIGHT_CAGE,
        RED_CENTER_CAGE,
        RED_REEF_CENTER,
        RED_REEF_A,
        RED_REEF_B,
        RED_REEF_C,
        RED_REEF_D,
        RED_REEF_E,
        RED_REEF_F,
        RED_REEF_G,
        RED_REEF_H,
        RED_REEF_I,
        RED_REEF_J,
        RED_REEF_K,
        RED_REEF_L,
        RED_BARGE_FRONT_CALCULATED,
        RED_BARGE_BACK_CALCULATED,
        RED_PROCESSOR_CALCULATED
    };

    enum class FIELD_ELEMENT_OFFSETS
    {
        LEFT_STICK,
        RIGHT_STICK
    };

    enum class CageLocation
    {
        LEFT,
        CENTER,
        RIGHT
    };

    frc::Pose3d GetFieldElementPose(FIELD_ELEMENT element);
    frc::Pose2d GetFieldElementPose2d(FIELD_ELEMENT element);

    frc::Pose3d GetAprilTagPose(FieldAprilTagIDs tag);
    frc::Pose2d GetAprilTagPose2d(FieldAprilTagIDs tag);

private:
    // make a singleton
    static FieldConstants *m_instance;
    std::vector<frc::AprilTag> m_aprilTagVector;
    const std::string m_fieldFilePath = "/home/lvuser/FieldData/output.json";
    // make constructor private
    FieldConstants();
    // make singleton copy constructor private
    FieldConstants(const FieldConstants &) = delete;
    FieldConstants &operator=(const FieldConstants &) = delete;
    frc::Pose3d GetAprilTagPoseFromLayout(int tagID);
    frc::AprilTagFieldLayout m_fieldLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025ReefscapeWelded); // change this guy for andymark field

    void ReadFieldCalibrationData();

    frc::Pose3d m_placeholder = frc::Pose3d();

    robin_hood::unordered_map<FIELD_ELEMENT, frc::Pose3d> fieldConstantsPoseMap;
    std::array<frc::Pose2d, 68> m_fieldConst2dPoses;

    robin_hood::unordered_map<int, frc::Pose3d> m_aprilTagPoseMap;
    std::array<frc::Pose2d, 23> m_aprilTag2dPoses;
};