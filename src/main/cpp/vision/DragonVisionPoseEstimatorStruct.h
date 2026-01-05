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
#include <vector>

#include "frc/geometry/Pose2d.h"
#include "units/time.h"
#include "wpi/array.h"

// Developer documentation:
// DragonVisionPoseEstimatorStruct
// -------------------------------
// Purpose:
//   Simple POD used to transfer a vision-based pose estimate from the vision
//   processing subsystem to the robot pose estimator. Contains the estimated
//   pose, a timestamp for when the measurement was taken, an enum-coded
//   confidence level for the measurement, and a small array of standard
//   deviations (uncertainties) for the x, y and heading components.
//
// Usage:
//   - Produced by vision code when a target is observed.
//   - Consumed by the pose estimator to fuse vision updates with other sensors.
//   - ConfidenceLevel should be used to gate or weight the measurement.
//   - m_stds: expected to be {std_x, std_y, std_theta} in meters/meters/radians
//     or consistent units with the rest of the estimator.
//
// Notes:
//   - Default constructor initializes to NONE confidence, zero pose and timestamp,
//     and a conservative default std array.
//
// Example:
//   DragonVisionPoseEstimatorStruct meas;
//   meas.m_confidenceLevel = DragonVisionPoseEstimatorStruct::HIGH;
//   meas.m_visionPose = somePose;
//   meas.m_timeStamp = units::time::second_t{frc::Timer::GetFPGATimestamp()};
//   passToEstimator(meas);
struct DragonVisionPoseEstimatorStruct
{
public:
    // Construct an empty/default measurement.
    // Defaults:
    //   m_confidenceLevel = NONE
    //   m_visionPose = identity / zero pose
    //   m_timeStamp = 0 seconds
    //   m_stds = {0.9, 0.9, 0.9} (conservative defaults)
    DragonVisionPoseEstimatorStruct() : m_confidenceLevel(ConfidenceLevel::NONE),
                                        m_visionPose(frc::Pose2d{}),
                                        m_timeStamp(units::time::second_t(0.0)),
                                        m_stds(wpi::array(0.9, 0.9, 0.9)) {};
    ~DragonVisionPoseEstimatorStruct() = default;

    // Confidence levels used by vision to indicate measurement quality.
    // - NONE:    No valid measurement available.
    // - LOW:     Measurement is available but low confidence (use cautiously).
    // - MEDIUM:  Typical measurement quality.
    // - HIGH:    High-quality measurement (preferable for fusion).
    enum ConfidenceLevel
    {
        NONE,
        LOW,
        MEDIUM,
        HIGH
    };

    // The confidence level for this measurement.
    ConfidenceLevel m_confidenceLevel;

    // Estimated robot pose (from vision) in field coordinates.
    frc::Pose2d m_visionPose;

    // Timestamp (seconds) indicating when the vision measurement was valid.
    // Use consistent timebase with other sensors when fusing.
    units::time::second_t m_timeStamp;

    // Standard deviations for the measurement: [std_x, std_y, std_theta].
    // Values should be in the same units as m_visionPose (meters/radians).
    wpi::array<double, 3> m_stds;
};
