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
#include "chassis/commands/season_specific_commands/DriveToBranch.h"
#include "fielddata/ReefHelper.h"

DriveToBranch::DriveToBranch(subsystems::CommandSwerveDrivetrain *chassis, FieldConstants::FIELD_ELEMENT_OFFSETS location)
    : DriveToPose(chassis),
      m_location(location)
{
}

frc::Pose2d DriveToBranch::GetEndPose()
{
    auto reefHelper = ReefHelper::GetInstance();
    auto taginfo = reefHelper->GetNearestReefTag();
    frc::Pose2d endPose;

    if (taginfo.has_value())
    {
        auto tag = taginfo.value();
        if (m_location == FieldConstants::FIELD_ELEMENT_OFFSETS::RIGHT_STICK)
        {
            auto rightBranch = ReefHelper::GetInstance()->GetNearestRightReefBranch(tag);
            if (rightBranch.has_value())
            {
                auto fieldconst = FieldConstants::GetInstance();
                auto rightBranchpose = fieldconst->GetFieldElementPose2d(rightBranch.value());
                endPose = frc::Pose2d(rightBranchpose.X(), rightBranchpose.Y(), frc::Rotation2d(rightBranchpose.Rotation().Degrees() + 180_deg)); // Have to add 180 degrees since the tag is facing the opposite direction of the robot
            }
        }
        else
        {
            auto leftBranch = ReefHelper::GetInstance()->GetNearestLeftReefBranch(tag);
            if (leftBranch.has_value())
            {
                auto fieldconst = FieldConstants::GetInstance();
                auto leftBranchPose = fieldconst->GetFieldElementPose2d(leftBranch.value());
                endPose = frc::Pose2d(leftBranchPose.X(), leftBranchPose.Y(), frc::Rotation2d(leftBranchPose.Rotation().Degrees() + 180_deg)); // Have to add 180 degrees since the tag is facing the opposite direction of the robot
            }
        }
    }
    return endPose;
}

void DriveToBranch::Execute()
{
    DriveToPose::Execute();

    auto reefHelper = ReefHelper::GetInstance();
    reefHelper->IsInZone();
}