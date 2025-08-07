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

// C++ Includes
#include <memory>
#include <string>

// Team 302 includes
#include "auton/drivePrimitives/AutonUtils.h"
#include "auton/drivePrimitives/IPrimitive.h"
#include "auton/drivePrimitives/ResetPositionPathPlanner.h"
#include "auton/PrimitiveParams.h"
#include "chassis/definitions/ChassisConfig.h"
#include "chassis/definitions/ChassisConfigMgr.h"
#include "chassis/SwerveChassis.h"
#include "vision/DragonVision.h"
#include "utils/logging/debug/Logger.h"
#include "utils/FMSData.h"

// Third Party Includes
#include "pathplanner/lib/path/PathPlannerPath.h"

using namespace std;
using namespace frc;
using namespace pathplanner;

ResetPositionPathPlanner::ResetPositionPathPlanner() : IPrimitive()
{
}

void ResetPositionPathPlanner::Init(PrimitiveParams *param)
{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;

    if (chassis != nullptr)
    {

        auto path = param->GetTrajectoryName().empty() ? AutonUtils::GetPathFromPathFile(param->GetPathName()) : AutonUtils::GetPathFromTrajectory(param->GetTrajectoryName());

        if (AutonUtils::IsValidPath(path))
        {
            auto initialPose = path.get()->getStartingHolonomicPose();
            if (initialPose)
            {
                // debugging
                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Reset Pose", "X", initialPose.value().X().value());
                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Reset Pose", "Y", initialPose.value().Y().value());
                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Reset Pose", "Theta", initialPose.value().Rotation().Degrees().value());

                // Check to see if current pose is within 2 meters (distanceThreshold) of the centerline (centerline), if it isn't, reset pose with pathplanner/choreo
                auto actualPose = chassis->GetPose();
                const units::length::meter_t poseDiff = units::math::abs(actualPose.X() - m_centerline);
                bool poseNeedsUpdating = poseDiff > m_distanceThreshold;

                if (poseNeedsUpdating)
                {
                    ResetPose(initialPose.value());
                }
            }
        }
    }
}

void ResetPositionPathPlanner::ResetPose(Pose2d pose)
{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;

    if (chassis != nullptr)
    {
        chassis->ResetPose(pose);
    }
}

void ResetPositionPathPlanner::Run()
{
}

bool ResetPositionPathPlanner::IsDone()
{
    return true;
}