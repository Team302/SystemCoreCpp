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

#include "vision/PoseOffsetUtils.h"
#include "units/math.h"

using namespace units::length;
using namespace units::angle;

std::pair<units::length::meter_t, units::length::meter_t> PoseOffsetUtils::CalculateXYDistanceFromObject(const DragonVisionStruct &target, units::length::inch_t objectHeight)
{
    const auto &odData = target.objectDetectionData;

    // 1. Calculate the ground distance from the camera lens to the object.
    //    Formula: Distance = (h_camera - h_object) / tan(pitch_camera + ty)
    //    We use the absolute value of the angle to ensure a positive distance, assuming the camera is facing the target.
    degree_t totalPitch = odData.camPitch + target.verticalOffset;
    meter_t zDelta = odData.mountingZOffset - objectHeight;

    // Avoid division by zero
    if (units::math::abs(totalPitch) < 0.01_deg)
    {
        totalPitch = 0.01_deg;
    }

    // Calculate ground distance
    meter_t distCameraToTargetGround = zDelta / units::math::tan(units::math::abs(totalPitch));

    // 2. Calculate the vector in the Camera's coordinate frame (X = Forward, Y = Left).
    //    Limelight tx is positive to the RIGHT.
    //    Therefore, Y relative to camera is -tan(tx) * distance
    meter_t camRelativeX = distCameraToTargetGround;
    meter_t camRelativeY = distCameraToTargetGround * -units::math::tan(target.horizontalOffset);

    // 3. Rotate this vector by the camera's mounting Yaw to get it into Robot-Relative orientation (but still at camera origin).
    //    Rotation (CCW positive):
    //    x' = x * cos(yaw) - y * sin(yaw)
    //    y' = x * sin(yaw) + y * cos(yaw)
    radian_t yaw = odData.camYaw;
    meter_t rotatedX = (camRelativeX * units::math::cos(yaw)) - (camRelativeY * units::math::sin(yaw));
    meter_t rotatedY = (camRelativeX * units::math::sin(yaw)) + (camRelativeY * units::math::cos(yaw));

    // 4. Add the Camera's mounting offset from the Robot Center.
    meter_t finalX = rotatedX + odData.mountingXOffset;
    meter_t finalY = rotatedY + odData.mountingYOffset;

    return {finalX, finalY};
}

units::length::meter_t PoseOffsetUtils::CalculateDistanceFromObject(const DragonVisionStruct &target, units::length::inch_t objectHeight)
{
    auto [x, y] = CalculateXYDistanceFromObject(target, objectHeight);
    return units::math::hypot(x, y);
}