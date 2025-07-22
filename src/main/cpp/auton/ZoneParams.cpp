
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

#include <frc/geometry/Pose2d.h>

#include "auton/AutonGrid.h"
#include "auton/ZoneParams.h"
#include "chassis/ChassisOptionEnums.h"

// @ADDMECH include for your mechanism state mgr

// @ADDMECH mechanism state for mech as parameter
// ZoneParams.cpp
#include "auton/ZoneParams.h"

ZoneParams::ZoneParams(
	frc::Pose2d circlePose,
	units::length::inch_t radius,
	units::length::meter_t xgrid1rect,
	units::length::meter_t xgrid2rect,
	units::length::meter_t ygrid1rect,
	units::length::meter_t ygrid2rect,
	bool isTaleStateChanging,
	DragonTale::STATE_NAMES taleOption,
	ChassisOptionEnums::AutonChassisOptions autonchassisoption,
	ChassisOptionEnums::HeadingOption headingOption,
	ChassisOptionEnums::DriveStateType pathUpdateOption,
	ChassisOptionEnums::AutonAvoidOptions autonavoidoption,
	ZoneMode zoneMode) : m_xgrid1rect(xgrid1rect),
						 m_xgrid2rect(xgrid2rect),
						 m_ygrid1rect(ygrid1rect),
						 m_ygrid2rect(ygrid2rect),
						 m_isTaleStateChanging(isTaleStateChanging),
						 m_taleOption(taleOption),
						 m_chassisoption(autonchassisoption),
						 m_headingOption(headingOption),
						 m_avoidoption(autonavoidoption),
						 m_pathUpdateOption(pathUpdateOption),
						 m_zoneMode(zoneMode),
						 m_circlePose(circlePose),
						 m_radius(radius)
{
}
bool ZoneParams::IsPoseInZone(frc::Pose2d robotPose)
{
	auto autonGrid = AutonGrid::GetInstance();
	switch (GetZoneMode())
	{
	case ZoneMode::RECTANGLE:
		return autonGrid->IsPoseInZone(GetX1Rect(), GetX2Rect(), GetY1Rect(), GetY2Rect(), robotPose);
	case ZoneMode::CIRCLE:
		return autonGrid->IsPoseInZone(GetCircleZonePose(), GetRadius(), robotPose);
	default:
		return true;
	}
}
