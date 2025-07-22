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

#include "frc/apriltag/AprilTagFieldLayout.h"
#include "RobinHood/robin_hood.h"
#include "RobinHood/robin_hood.h"
#include "units/angle.h"
#include "units/base.h"

#define WELDED_FIELD // comment this guy out when not welded field :)

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

    enum FIELD_ELEMENT_OFFSETS
    {
        LEFT_STICK,
        RIGHT_STICK
    };

    enum AprilTagIDs
    {
        // Blue
        BLUE_CORAL_STATION_LEFT_TAG = 13,
        BLUE_CORAL_STATION_RIGHT_TAG = 12,
        BLUE_PROCESSOR_TAG = 16,
        BLUE_BARGE_FRONT_TAG = 14,
        BLUE_BARGE_BACK_TAG = 4,
        BLUE_REEF_AB_TAG = 18,
        BLUE_REEF_CD_TAG = 17,
        BLUE_REEF_EF_TAG = 22,
        BLUE_REEF_GH_TAG = 21,
        BLUE_REEF_IJ_TAG = 20,
        BLUE_REEF_KL_TAG = 19,
        // Red
        RED_CORAL_STATION_LEFT_TAG = 1,
        RED_CORAL_STATION_RIGHT_TAG = 2,
        RED_PROCESSOR_TAG = 3,
        RED_BARGE_FRONT_TAG = 5,
        RED_BARGE_BACK_TAG = 15,
        RED_REEF_AB_TAG = 7,
        RED_REEF_CD_TAG = 8,
        RED_REEF_EF_TAG = 9,
        RED_REEF_GH_TAG = 10,
        RED_REEF_IJ_TAG = 11,
        RED_REEF_KL_TAG = 6
    };
    frc::Pose3d GetFieldElementPose(FIELD_ELEMENT element);
    frc::Pose2d GetFieldElementPose2d(FIELD_ELEMENT element);

    frc::Pose3d GetAprilTagPose(AprilTagIDs tag);
    frc::Pose2d GetAprilTagPose2d(AprilTagIDs tag);

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

    void ReadFieldCalibrationData();

    // specified here
    // https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025LayoutMarkingDiagram.pdf

    // TODO: JW - took values blindly from table, there was only a Z and Y rotation, so we probably need to calculate the X rotation
    // TODO: JW - these values are the defaults, but should be able to be set/overridden from WpiCal json file

#ifdef WELDED_FIELD
    frc::Pose3d m_aprilTag1 = frc::Pose3d(
        units::length::inch_t(657.37),
        units::length::inch_t(25.8),
        units::length::inch_t(58.5),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(126, 0)));

    frc::Pose3d m_aprilTag2 = frc::Pose3d(
        units::length::inch_t(657.37),
        units::length::inch_t(291.2),
        units::length::inch_t(58.5),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(234.0)));

    frc::Pose3d m_aprilTag3 = frc::Pose3d(
        units::length::inch_t(455.15),
        units::length::inch_t(317.15),
        units::length::inch_t(51.25),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(270.0)));

    frc::Pose3d m_aprilTag4 = frc::Pose3d(
        units::length::inch_t(365.20),
        units::length::inch_t(241.64),
        units::length::inch_t(73.54),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(30.0), units::angle::degree_t(0.0)));

    frc::Pose3d m_aprilTag5 = frc::Pose3d(
        units::length::inch_t(365.20),
        units::length::inch_t(75.39),
        units::length::inch_t(73.54),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(30.0), units::angle::degree_t(0.0)));

    frc::Pose3d m_aprilTag6 = frc::Pose3d(
        units::length::inch_t(530.49),
        units::length::inch_t(130.17),
        units::length::inch_t(12.13),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(300.0)));

    frc::Pose3d m_aprilTag7 = frc::Pose3d(
        units::length::inch_t(546.87),
        units::length::inch_t(158.50),
        units::length::inch_t(12.13),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(0.0)));

    frc::Pose3d m_aprilTag8 = frc::Pose3d(
        units::length::inch_t(530.49),
        units::length::inch_t(186.83),
        units::length::inch_t(12.13),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(60.0)));

    frc::Pose3d m_aprilTag9 = frc::Pose3d(
        units::length::inch_t(497.77),
        units::length::inch_t(186.83),
        units::length::inch_t(12.13),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(120.0)));

    frc::Pose3d m_aprilTag10 = frc::Pose3d(
        units::length::inch_t(481.39),
        units::length::inch_t(158.50),
        units::length::inch_t(12.13),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(180.0)));

    frc::Pose3d m_aprilTag11 = frc::Pose3d(
        units::length::inch_t(497.77),
        units::length::inch_t(130.17),
        units::length::inch_t(12.13),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(240.0)));

    frc::Pose3d m_aprilTag12 = frc::Pose3d(
        units::length::inch_t(33.51),
        units::length::inch_t(25.80),
        units::length::inch_t(58.50),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(54.0)));

    frc::Pose3d m_aprilTag13 = frc::Pose3d(
        units::length::inch_t(33.51),
        units::length::inch_t(291.20),
        units::length::inch_t(58.50),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(306.0)));

    frc::Pose3d m_aprilTag14 = frc::Pose3d(
        units::length::inch_t(325.68),
        units::length::inch_t(241.64),
        units::length::inch_t(73.54),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(30.0), units::angle::degree_t(180.0)));

    frc::Pose3d m_aprilTag15 = frc::Pose3d(
        units::length::inch_t(325.68),
        units::length::inch_t(75.39),
        units::length::inch_t(73.54),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(30.0), units::angle::degree_t(180.0)));

    frc::Pose3d m_aprilTag16 = frc::Pose3d(
        units::length::inch_t(235.73),
        units::length::inch_t(-0.15),
        units::length::inch_t(51.25),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(90.0)));

    frc::Pose3d m_aprilTag17 = frc::Pose3d(
        units::length::inch_t(160.39),
        units::length::inch_t(130.17),
        units::length::inch_t(12.13),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(240.0)));

    frc::Pose3d m_aprilTag18 = frc::Pose3d(
        units::length::inch_t(144.00),
        units::length::inch_t(158.50),
        units::length::inch_t(12.13),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(180.0)));

    frc::Pose3d m_aprilTag19 = frc::Pose3d(
        units::length::inch_t(160.39),
        units::length::inch_t(186.83),
        units::length::inch_t(12.13),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(120.0)));

    frc::Pose3d m_aprilTag20 = frc::Pose3d(
        units::length::inch_t(193.10),
        units::length::inch_t(186.83),
        units::length::inch_t(12.13),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(60.0)));

    frc::Pose3d m_aprilTag21 = frc::Pose3d(
        units::length::inch_t(209.49),
        units::length::inch_t(158.50),
        units::length::inch_t(12.13),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(0.0)));

    frc::Pose3d m_aprilTag22 = frc::Pose3d(
        units::length::inch_t(193.10),
        units::length::inch_t(130.17),
        units::length::inch_t(12.13),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(300.0)));

#else

    frc::Pose3d m_aprilTag1 = frc::Pose3d(
        units::length::inch_t(656.98),
        units::length::inch_t(24.73),
        units::length::inch_t(58.5),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(126, 0)));

    frc::Pose3d m_aprilTag2 = frc::Pose3d(
        units::length::inch_t(656.98),
        units::length::inch_t(291.9),
        units::length::inch_t(58.5),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(234.0)));

    frc::Pose3d m_aprilTag3 = frc::Pose3d(
        units::length::inch_t(452.4),
        units::length::inch_t(316.21),
        units::length::inch_t(51.25),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(270.0)));

    frc::Pose3d m_aprilTag4 = frc::Pose3d(
        units::length::inch_t(365.20),
        units::length::inch_t(241.44),
        units::length::inch_t(73.54),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(30.0), units::angle::degree_t(0.0)));

    frc::Pose3d m_aprilTag5 = frc::Pose3d(
        units::length::inch_t(365.20),
        units::length::inch_t(75.19),
        units::length::inch_t(73.54),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(30.0), units::angle::degree_t(0.0)));

    frc::Pose3d m_aprilTag6 = frc::Pose3d(
        units::length::inch_t(530.49),
        units::length::inch_t(129.97),
        units::length::inch_t(12.13),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(300.0)));

    frc::Pose3d m_aprilTag7 = frc::Pose3d(
        units::length::inch_t(546.87),
        units::length::inch_t(158.30),
        units::length::inch_t(12.13),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(0.0)));

    frc::Pose3d m_aprilTag8 = frc::Pose3d(
        units::length::inch_t(530.49),
        units::length::inch_t(186.63),
        units::length::inch_t(12.13),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(60.0)));

    frc::Pose3d m_aprilTag9 = frc::Pose3d(
        units::length::inch_t(497.77),
        units::length::inch_t(186.63),
        units::length::inch_t(12.13),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(120.0)));

    frc::Pose3d m_aprilTag10 = frc::Pose3d(
        units::length::inch_t(481.39),
        units::length::inch_t(158.30),
        units::length::inch_t(12.13),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(180.0)));

    frc::Pose3d m_aprilTag11 = frc::Pose3d(
        units::length::inch_t(497.77),
        units::length::inch_t(129.97),
        units::length::inch_t(12.13),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(240.0)));

    frc::Pose3d m_aprilTag12 = frc::Pose3d(
        units::length::inch_t(33.91),
        units::length::inch_t(24.73),
        units::length::inch_t(58.50),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(54.0)));

    frc::Pose3d m_aprilTag13 = frc::Pose3d(
        units::length::inch_t(33.91),
        units::length::inch_t(291.90),
        units::length::inch_t(58.50),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(306.0)));

    frc::Pose3d m_aprilTag14 = frc::Pose3d(
        units::length::inch_t(325.68),
        units::length::inch_t(241.44),
        units::length::inch_t(73.54),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(30.0), units::angle::degree_t(180.0)));

    frc::Pose3d m_aprilTag15 = frc::Pose3d(
        units::length::inch_t(325.68),
        units::length::inch_t(75.19),
        units::length::inch_t(73.54),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(30.0), units::angle::degree_t(180.0)));

    frc::Pose3d m_aprilTag16 = frc::Pose3d(
        units::length::inch_t(238.49),
        units::length::inch_t(0.42),
        units::length::inch_t(51.25),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(90.0)));

    frc::Pose3d m_aprilTag17 = frc::Pose3d(
        units::length::inch_t(160.39),
        units::length::inch_t(129.97),
        units::length::inch_t(12.13),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(240.0)));

    frc::Pose3d m_aprilTag18 = frc::Pose3d(
        units::length::inch_t(144.00),
        units::length::inch_t(158.30),
        units::length::inch_t(12.13),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(180.0)));

    frc::Pose3d m_aprilTag19 = frc::Pose3d(
        units::length::inch_t(160.39),
        units::length::inch_t(186.63),
        units::length::inch_t(12.13),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(120.0)));

    frc::Pose3d m_aprilTag20 = frc::Pose3d(
        units::length::inch_t(193.10),
        units::length::inch_t(186.63),
        units::length::inch_t(12.13),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(60.0)));

    frc::Pose3d m_aprilTag21 = frc::Pose3d(
        units::length::inch_t(209.49),
        units::length::inch_t(158.30),
        units::length::inch_t(12.13),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(0.0)));

    frc::Pose3d m_aprilTag22 = frc::Pose3d(
        units::length::inch_t(193.10),
        units::length::inch_t(129.97),
        units::length::inch_t(12.13),
        frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::angle::degree_t(300.0)));

#endif

    frc::Pose3d m_placeholder = frc::Pose3d();

    robin_hood::unordered_map<FIELD_ELEMENT, frc::Pose3d> fieldConstantsPoseMap;
    std::array<frc::Pose2d, 68> m_fieldConst2dPoses;

    robin_hood::unordered_map<int, frc::Pose3d> m_aprilTagPoseMap;
    std::array<frc::Pose2d, 23> m_aprilTag2dPoses;
};