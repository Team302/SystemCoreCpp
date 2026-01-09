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

#include <memory>
#include <utility>
#include <cmath>

#include "units/length.h"
#include "units/angle.h"
#include "vision/DragonVisionStruct.h"

class PoseOffsetUtils
{
public:
    PoseOffsetUtils() = delete;
    ~PoseOffsetUtils() = delete;

    /// @brief Calculates the X and Y distance from the center of the robot to the detected object.
    /// @param target The vision target data (ObjectDetection).
    /// @return std::pair<units::length::meter_t, units::length::meter_t> where first is X (Forward), second is Y (Left).
    static std::pair<units::length::meter_t, units::length::meter_t> CalculateXYDistanceFromObject(const DragonVisionStruct &target, units::length::inch_t objectHeight);

    /// @brief Calculates the straight-line ground distance (hypotenuse) from the center of the robot to the object.
    /// @param target The vision target data (ObjectDetection).
    /// @return units::length::meter_t The total ground distance.
    static units::length::meter_t CalculateDistanceFromObject(const DragonVisionStruct &target, units::length::inch_t objectHeight);
};