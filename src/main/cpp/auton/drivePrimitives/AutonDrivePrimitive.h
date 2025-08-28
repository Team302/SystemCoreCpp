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
#pragma once

#include "auton/drivePrimitives/IPrimitive.h"
#include "frc/Timer.h"
#include "frc2/command/Command.h"
#include <frc2/command/CommandScheduler.h>
#include "auton/ZoneParams.h"
#include "chassis/generated/CommandSwerveDrivetrain.h"
#include "mechanisms/DragonTale/DragonTale.h"
#include "auton/PrimitiveEnums.h"

class AutonDrivePrimitive : public IPrimitive
{
public:
    AutonDrivePrimitive();
    ~AutonDrivePrimitive() = default;

    void Init(PrimitiveParams *params) override;
    void Run() override;
    bool IsDone() override;

private:
    frc2::CommandPtr CreateDriveToTargetCommand(ChassisOptionEnums::DriveStateType driveToType);
    bool IsInZone();
    int FindDriveToZoneIndex(ZoneParamsVector zones);

    subsystems::CommandSwerveDrivetrain *m_chassis;
    std::unique_ptr<frc::Timer> m_timer;
    frc2::CommandPtr m_managedCommand;

    PRIMITIVE_IDENTIFIER m_activeId;
    units::time::second_t m_maxTime;
    bool m_visionTransition;
    bool m_checkForDriveToUpdate;
    ZoneParams *m_zone;
    DragonTale *m_dragonTaleMgr;
};