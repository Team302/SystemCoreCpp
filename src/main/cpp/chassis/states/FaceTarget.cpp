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

#include <tuple>

// Team302 Includes
#include "fielddata/DragonTargetFinder.h"
#include "chassis/definitions/ChassisConfigMgr.h"
#include "chassis/states/FaceTarget.h"
#include "frc/geometry/Pose2d.h"
#include "utils/AngleUtils.h"

FaceTarget::FaceTarget(ChassisOptionEnums::HeadingOption headingOption) : SpecifiedHeading(headingOption), m_chassis(ChassisConfigMgr::GetInstance()->GetCurrentChassis())
{
}

std::string FaceTarget::GetHeadingStateName() const
{
    return std::string("FaceTarget");
}

units::angle::degree_t FaceTarget::GetTargetAngle(ChassisMovement &chassisMovement) const
{
    if (m_chassis != nullptr)
    {
        auto chassispose = m_chassis->GetPose();
        auto currentangle = m_chassis->GetStoredHeading();

        auto finder = DragonTargetFinder::GetInstance();
        if (finder != nullptr)
        {
            auto info = finder->GetPose(GetTarget());
            if (info.has_value())
            {
                auto targetpose = get<1>(info.value());

                units::length::meter_t xDiff = targetpose.X() - chassispose.X();
                units::length::meter_t yDiff = targetpose.Y() - chassispose.Y();
                units::angle::degree_t angletotarget = units::math::atan2(yDiff, xDiff);

                // Adjust angleToReefCenter to be between -180 and 180 degrees
                angletotarget = AngleUtils::GetEquivAngle(angletotarget);
                chassisMovement.yawAngle = angletotarget;
                return angletotarget;
            }
        }
        return currentangle;
    }
    return units::angle::degree_t(0.0);
}
