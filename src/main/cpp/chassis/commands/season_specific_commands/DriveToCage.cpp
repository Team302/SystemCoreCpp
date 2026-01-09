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
#include "chassis/commands/season_specific_commands/DriveToCage.h"
#include "fielddata/FieldConstants.h"
#include "utils/FMSData.h"

DriveToCage::DriveToCage(subsystems::CommandSwerveDrivetrain *chassis, FieldConstants::CageLocation location)
    : DriveToPose(chassis), m_location(location)
{
}

frc::Pose2d DriveToCage::GetEndPose()
{
    auto allianceColor = FMSData::GetAllianceColor();
    auto fieldElement = allianceColor == frc::DriverStation::kRed ? FieldConstants::FIELD_ELEMENT::RED_BARGE_FRONT_CALCULATED : FieldConstants::FIELD_ELEMENT::BLUE_BARGE_FRONT_CALCULATED; // defualt value in front of barge

    if (m_location == FieldConstants::CageLocation::LEFT)
        fieldElement = allianceColor == frc::DriverStation::kRed ? FieldConstants::FIELD_ELEMENT::RED_LEFT_CAGE : FieldConstants::FIELD_ELEMENT::BLUE_LEFT_CAGE;
    else if (m_location == FieldConstants::CageLocation::RIGHT)
        fieldElement = allianceColor == frc::DriverStation::kRed ? FieldConstants::FIELD_ELEMENT::RED_RIGHT_CAGE : FieldConstants::FIELD_ELEMENT::BLUE_RIGHT_CAGE;
    else if (m_location == FieldConstants::CageLocation::CENTER)
        fieldElement = allianceColor == frc::DriverStation::kRed ? FieldConstants::FIELD_ELEMENT::RED_CENTER_CAGE : FieldConstants::FIELD_ELEMENT::BLUE_CENTER_CAGE;

    auto m_cagePose = FieldConstants::GetInstance()->GetFieldElementPose2d(fieldElement);

    return frc::Pose2d(m_cagePose.X(), m_cagePose.Y(), frc::Rotation2d(m_cagePose.Rotation().Degrees() + 90_deg));
}
