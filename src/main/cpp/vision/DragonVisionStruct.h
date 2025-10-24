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
//====================================================================================================================================================#pragma once
#pragma once

#include "units/angle.h"
#include "units/length.h"
#include "units/time.h"
#include <string>

#include "vision/DragonVisionEnums.h"

struct DragonVisionStruct
{
    int targetID = -1;
    DragonTargetType targetType = DragonTargetType::UNKNOWN;
    std::string className = ""; // only used by Machine Learning
    units::angle::degree_t horizontalOffset = 0_deg;
    units::angle::degree_t verticalOffset = 0_deg;
    double targetAreaPercent = 0.0;
    units::time::millisecond_t pipelineLatency = 0_ms; // should be tl + cl
    units::length::meter_t distanceToCamera = 0_m;
    units::length::meter_t distanceToRobot = 0_m;
    double ambiguity = 1.0; // only april tags
};
