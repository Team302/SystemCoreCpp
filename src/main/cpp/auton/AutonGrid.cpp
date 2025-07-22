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

// FRC Includes
#include <math.h>

#include <frc/geometry/Pose2d.h>

// Team302 Includes
#include "auton/AutonGrid.h"

// Thirdparty includes

AutonGrid *AutonGrid::m_instance = nullptr; // initialize m_instance as a nullptr

AutonGrid *AutonGrid::GetInstance()
{
    // if m_instance is nullptr then a new instance of AutonGrid is created and returned therefore only leaving one instance of the class
    if (AutonGrid::m_instance == nullptr)
    {
        AutonGrid::m_instance = new AutonGrid();
    }
    return AutonGrid::m_instance;
} // to make the class a singlton

bool AutonGrid::IsPoseInZone(units::length::meter_t xgrid1, units::length::meter_t xgrid2, units::length::meter_t ygrid1, units::length::meter_t ygrid2, frc::Pose2d robotPose)
// defining IsPoseInZone bool method and pulling in the arguements
{

    // then it is determined whether or not the robotPose is in the zone defined by the 2 grids.
    // TODO: remove dependency of order of x's and y's
    return ((robotPose.X() >= xgrid1) && (robotPose.X() <= xgrid2) &&
            (robotPose.Y() >= ygrid1) && (robotPose.Y() <= ygrid2));
}
bool AutonGrid::IsPoseInZone(frc::Pose2d circleZonePose, units::length::inch_t radius, frc::Pose2d robotPose)
{
    auto translationToCenter = circleZonePose.Translation().Distance(robotPose.Translation());
    bool inZone = translationToCenter <= radius;

    return inZone;
}
