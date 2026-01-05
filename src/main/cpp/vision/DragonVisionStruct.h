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

#include "units/angle.h"
#include "units/length.h"
#include "units/time.h"
#include <string>

#include "fielddata/FieldAprilTagIDs.h"
#include "vision/DragonVisionEnums.h"

/// @brief Per-detection data specific to AprilTag detections.
/// @details Contains ids and distance estimates produced/consumed by Limelight/APRILTAG pipelines.
///          Distances are stored as meters (units::length::meter_t). Ambiguity is a confidence metric
///          reported by the vision system (higher = more ambiguous).
struct AprilTagSpecificData
{
    FieldAprilTagIDs tagID;                    ///< AprilTag ID enum (field reference)
    units::length::meter_t distToCamera = 0_m; ///< Distance from camera to tag in meters
    units::length::meter_t distToRobot = 0_m;  ///< Distance from robot center to tag in meters
    double ambiguity = 1.0;                    ///< Detection ambiguity/confidence (implementation-specific)
};

/// @brief Per-detection data specific to object-detection results (neural or shape detectors).
/// @details Stores the detected class id and the four polygon corner coordinates returned by the detector.
///          Corner coordinates are typically normalized or image-pixel values depending on the detector.
struct ObjectDectcionSpecificData
{
    int classID = -1;      ///< Class identifier returned by the detector (-1 = unknown)
    double corner0X = 0.0; ///< X coordinate of corner 0 (units depend on detector output)
    double corner0Y = 0.0; ///< Y coordinate of corner 0
    double corner1X = 0.0; ///< X coordinate of corner 1
    double corner1Y = 0.0; ///< Y coordinate of corner 1
    double corner2X = 0.0; ///< X coordinate of corner 2
    double corner2Y = 0.0; ///< Y coordinate of corner 2
    double corner3X = 0.0; ///< X coordinate of corner 3
    double corner3Y = 0.0; ///< Y coordinate of corner 3
};

/// @brief Generic vision detection container used by Dragon vision subsystems.
/// @details Combines common detection fields (offsets, area, latency) with type-specific data
///          in aprilTagData or objectDetectionData. Offsets use units::angle::degree_t, latency
///          uses units::time::millisecond_t.
struct DragonVisionStruct
{
    DragonTargetType targetType = DragonTargetType::UNKNOWN; ///< Detection type enum
    units::angle::degree_t horizontalOffset = 0_deg;         ///< Horizontal angular offset to target
    units::angle::degree_t verticalOffset = 0_deg;           ///< Vertical angular offset to target
    double targetAreaPercent = 0.0;                          ///< Target area as reported by vision (fraction)
    units::time::millisecond_t pipelineLatency = 0_ms;       ///< Pipeline + capture latency estimate
    AprilTagSpecificData aprilTagData;                       ///< AprilTag-specific values (if targetType==APRIL_TAG)
    ObjectDectcionSpecificData objectDetectionData;          ///< Object-detection-specific values (if targetType==OBJECT_DETECTION)
};
