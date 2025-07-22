
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

#include "chassis/ChassisConfigMgr.h"
#include "fielddata/ProcessorHelper.h"
#include "utils/FMSData.h"

ProcessorHelper *ProcessorHelper::m_instance = nullptr;
ProcessorHelper *ProcessorHelper::GetInstance()
{
    if (ProcessorHelper::m_instance == nullptr)
    {
        ProcessorHelper::m_instance = new ProcessorHelper();
    }
    return ProcessorHelper::m_instance;
}

ProcessorHelper::ProcessorHelper() : m_chassis(ChassisConfigMgr::GetInstance()->GetSwerveChassis()),
                                     m_fieldConstants(FieldConstants::GetInstance())
{
}

frc::Pose2d ProcessorHelper::CalcProcessorPose()
{
    auto allianceColor = FMSData::GetAllianceColor();
    frc::Pose2d pose2d{};
    if (allianceColor == frc::DriverStation::Alliance::kRed)
    {
        pose2d = m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::RED_PROCESSOR_CALCULATED);
    }
    else
    {
        pose2d = m_fieldConstants->GetFieldElementPose2d(FieldConstants::FIELD_ELEMENT::BLUE_PROCESSOR_CALCULATED);
    }
    return frc::Pose2d(pose2d.X(), pose2d.Y(), pose2d.Rotation().Degrees() + units::degree_t(180));
}
std::optional<FieldConstants::AprilTagIDs> ProcessorHelper::GetAprilTag()
{
    return FMSData::GetAllianceColor() == frc::DriverStation::Alliance::kRed ? FieldConstants::AprilTagIDs::RED_PROCESSOR_TAG : FieldConstants::AprilTagIDs::BLUE_PROCESSOR_TAG;
}
