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

#include <string>
#include "vision/DragonVisionStructs.h"
#include "vision/DragonLimelight.h"
#include "frc/geometry/Translation3d.h"
#include "frc/geometry/Transform3d.h"
#include "Limelight/LimelightHelpers.h"

class DragonVisionStructLogger
{
public:
    static void logVisionData(const std::string& loggerName, const std::optional<VisionData> optVisionData);
    static void logTransform3d(const std::string &loggerName, const frc::Transform3d transform3d);
    static void logTranslation3d(const std::string &loggerName, const frc::Translation3d translation3d);
    static void logRotation3d(const std::string &loggerName, const frc::Rotation3d rotation3d);
    static void logDragonCamera(const std::string &loggerName, const DragonLimelight& camera);
    static void logPose3d(const std::string &loggerName, const frc::Pose3d pose3d);
    static void logVisionPose(const std::string &loggerName, const std::optional<VisionPose> optVisionPose);
    static void logPose2d(const std::string &loggerName, const frc::Pose2d pose2d);
    static void logLLPoseEstimation(const std::string &loggerName, const LimelightHelpers::PoseEstimate llPoseEstimate);
};