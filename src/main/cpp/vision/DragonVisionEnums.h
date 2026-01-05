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

// Developer documentation:
// @file DragonVisionEnums.h
// @brief Enumerations for the vision subsystem.
//
// Purpose:
//   Central location for strongly-typed enums used by vision-related code
//   (target selection, camera types, modes, pipelines, and pose strategy).
//
// Usage:
//   - These enums are referenced by vision processing modules, camera wrappers,
//     and higher-level pose estimation/selection logic.
//   - Keep enum values stable (don't reorder) if they are serialized/shared
//     across networktables or logs.
//
// Enum summaries:
//   - VisionTargetOption: How to select targets from the vision pipeline.
//   - DragonTargetType: Type of detected target (AprilTag, object detection, ...).
//   - DRAGON_LIMELIGHT_CAMERA_TYPE: Hardware model variant of Limelight used.
//   - DRAGON_LIMELIGHT_CAMERA_IDENTIFIER: Which camera on the robot (front/back).
//   - DRAGON_LIMELIGHT_CAMERA_USAGE: Primary intended usage of the camera.
//   - DRAGON_LIMELIGHT_LED_MODE: LED control modes exposed by Limelight.
//   - DRAGON_LIMELIGHT_CAM_MODE: Camera mode (vision processing vs driver view).
//   - DRAGON_LIMELIGHT_STREAM_MODE: Multi-camera stream layout options.
//   - DRAGON_LIMELIGHT_SNAPSHOT_MODE: Snapshot (still image) capture mode.
//   - DRAGON_LIMELIGHT_PIPELINE: Named processing pipelines (tag/detection).
//   - PoseEstimationStrategy: Strategy used when combining tag observations
//     into a pose estimate (multi-tag, single-tag, mega-tag variants).
//
// Notes:
//   - Prefer using these enums instead of raw integers/strings when interacting
//     with vision APIs to improve clarity and type-safety.
//   - Add short comments near enums if you add new values to document intended
//     behaviour and any external mapping (NT keys, persisted values).

enum class VisionTargetOption
{
    CLOSEST_VALID_TARGET,
    FUSED_TARGET_INFO,
    ALL_VALID_TARGETS
};

enum class DragonTargetType
{
    UNKNOWN,
    APRIL_TAG,
    OBJECT_DETECTION
};

enum class DRAGON_LIMELIGHT_CAMERA_TYPE
{
    LIMELIGHT4,
    LIMELIGHT4_W_HAILO8,
    LIMELIGHT3G,
    LIMELIGHT3,
    LIMELIGHT3_W_CORAL
};

enum class DRAGON_LIMELIGHT_CAMERA_IDENTIFIER
{
    BACK_CAMERA,
    FRONT_CAMERA
};

enum class DRAGON_LIMELIGHT_CAMERA_USAGE
{
    APRIL_TAGS,
    OBJECT_DETECTION_ALGAE,
    ALGAE_AND_APRIL_TAGS
};

enum class DRAGON_LIMELIGHT_LED_MODE
{
    LED_UNKNOWN = -1,
    LED_PIPELINE_CONTROL,
    LED_OFF,
    LED_BLINK,
    LED_ON
};

enum class DRAGON_LIMELIGHT_CAM_MODE
{
    CAM_UNKNOWN = -1,
    CAM_VISION,
    CAM_DRIVER
};

enum class DRAGON_LIMELIGHT_STREAM_MODE
{
    STREAM_UNKNOWN = -1,
    STREAM_STANDARD,     // side by side if two cams
    STREAM_PIP_MAIN,     // Second Cam bottom right of Main Cam
    STREAM_PIP_SECONDARY // Main Cam bottom right of Second Cam
};

enum class DRAGON_LIMELIGHT_SNAPSHOT_MODE
{
    SNAPSHOT_MODE_UNKNOWN = -1,
    SNAP_OFF,
    SNAP_ON
};

enum class DRAGON_LIMELIGHT_PIPELINE
{
    UNKNOWN = -1,
    APRIL_TAG = 0,
    MACHINE_LEARNING_PL = 1,
    COLOR_THRESHOLD
};

enum class PoseEstimationStrategy
{
    MULTI_TAG,
    SINGLE_TAG,
    MEGA_TAG,
    MEGA_TAG_2,
    NONE
};
