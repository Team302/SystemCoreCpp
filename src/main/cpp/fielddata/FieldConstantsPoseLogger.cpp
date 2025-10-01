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

#include <fielddata/FieldConstantsPoseLogger.h>

#ifdef INCLUDE_FIELD_ELEMENT_POSE_LOGGER
#include "wpi/datalog/DataLog.h"
#include "frc/DataLogManager.h"
#include "frc/geometry/Pose3d.h"
#include "magic_enum/magic_enum.hpp"

void FieldConstantsPoseLogger::LogFieldElementPoses(robin_hood::unordered_map<FieldConstants::FIELD_ELEMENT, frc::Pose3d> &fieldConstantsPoseMap)
{

    frc::DataLogManager::Start("", "field_poses.wpilog");
    wpi::log::DataLog &log = frc::DataLogManager::GetLog();

    for (auto &[key, pose] : fieldConstantsPoseMap)
    {
        auto poseLog = wpi::log::StructLogEntry<frc::Pose3d>(log, magic_enum::enum_name(key));
        poseLog.Append(pose);
    }

    log.Flush();
}

#endif
