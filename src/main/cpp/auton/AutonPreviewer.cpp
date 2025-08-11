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

#include <string>

// FRC Includes
#include "frc/trajectory/Trajectory.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/trajectory/Trajectory.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/velocity.h"

// Team302 Includes
#include "auton/AutonPreviewer.h"
#include "auton/AutonSelector.h"
#include "auton/PrimitiveEnums.h"
#include "auton/PrimitiveParams.h"
#include "auton/PrimitiveParser.h"
#include "utils/logging/debug/Logger.h"
#include "chassis/definitions/ChassisConfigMgr.h"
#include "chassis/SwerveChassis.h"

using frc::ChassisSpeeds;
using frc::Rotation2d;
using frc::Trajectory;
using std::string;

AutonPreviewer::AutonPreviewer(CyclePrimitives *cyclePrims) : m_selector(cyclePrims->GetAutonSelector()),
                                                              m_prevChoice(""),
                                                              m_field(DragonField::GetInstance())
{
}

void AutonPreviewer::CheckCurrentAuton()
{
    std::string currentChoice = m_selector->GetSelectedAutoFile();
    if (currentChoice != m_prevChoice)
    {
        PopulateField();
        m_prevChoice = currentChoice;
    }
}

void AutonPreviewer::PopulateField()
{

    m_field->ResetField();
}
