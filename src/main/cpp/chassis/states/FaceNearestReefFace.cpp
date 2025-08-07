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

// Team302 Includes
#include "chassis/states/FaceNearestReefFace.h"
#include "utils/logging/debug/Logger.h"
#include "vision/DragonVision.h"

FaceNearestReefFace::FaceNearestReefFace() : FaceTarget(ChassisOptionEnums::HeadingOption::FACE_REEF_CENTER)
{
}

std::string FaceNearestReefFace::GetHeadingStateName() const
{
    return std::string("FaceNearestReefFace");
}

DragonTargetFinderTarget FaceNearestReefFace::GetTarget() const
{
    return DragonTargetFinderTarget::CLOSEST_REEF_ALGAE; // there is no enum for reef face, so we use reef algae instead.. which is the same thing
}
units::angle::degree_t FaceNearestReefFace::GetTargetAngle(ChassisMovement &chassisMovement) const
{
    auto finder = DragonTargetFinder::GetInstance();
    if (finder != nullptr)
    {
        auto info = finder->GetPose(GetTarget());
        if (info.has_value())
        {
            auto targetpose = get<1>(info.value());
            DragonTargetFinderData type = get<0>(info.value());

            chassisMovement.yawAngle = (type == DragonTargetFinderData::ODOMETRY_BASED) ? targetpose.Rotation().Degrees() - 180_deg : targetpose.Rotation().Degrees();

            return chassisMovement.yawAngle;
        }
    }

    return units::angle::degree_t(0);
}
