
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

// #include <array>
#include <map>

#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Pose3d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "wpi/datalog/DataLog.h"

class DragonDataLoggerSignals
{
    friend class DragonDataLogger;

public:
    static DragonDataLoggerSignals *GetInstance();

private:
    // initialize these signals in the constructor

    std::string m_brownOutPath = "";

    std::string m_storedHeadingPath = "";
    std::string m_ChassisYawPath = "";

    std::string m_electricalVoltagePath = "";
    std::string m_electricalCurrentPath = "";
    std::string m_electricalEnergyPath = "";
    std::string m_electricalPowerPath = "";

    std::string m_txPath = "";
    std::string m_tyPath = "";
    std::string m_tvPath = "";
    std::string m_fiducialPath = "";

    std::string m_batteryVoltagePath = "";
    std::string m_brownoutVoltagePath = "";
    std::string m_inputVoltagePath = "";
    std::string m_inputCurrentPath = "";
    std::string m_cpuTempPath = "";

    std::string m_lfSteerPowerPath = "";
    std::string m_lfSteerEnergyPath = "";
    std::string m_lfSteerTotalPowerPath = "";
    std::string m_lfSteerWattHoursPath = "";

    std::string m_lfDrivePowerPath = "";
    std::string m_lfDriveEnergyPath = "";
    std::string m_lfDriveTotalPowerPath = "";
    std::string m_lfDriveWattHoursPath = "";
    std::string m_rfSteerPowerPath = "";
    std::string m_rfSteerEnergyPath = "";
    std::string m_rfSteerTotalPowerPath = "";
    std::string m_rfSteerWattHoursPath = "";
    std::string m_rfDrivePowerPath = "";
    std::string m_rfDriveEnergyPath = "";
    std::string m_rfDriveTotalPowerPath = "";
    std::string m_rfDriveWattHoursPath = "";

    std::string m_lbSteerPowerPath = "";
    std::string m_lbSteerEnergyPath = "";
    std::string m_lbSteerTotalPowerPath = "";
    std::string m_lbSteerWattHoursPath = "";
    std::string m_lbDrivePowerPath = "";
    std::string m_lbDriveEnergyPath = "";
    std::string m_lbDriveTotalPowerPath = "";
    std::string m_lbDriveWattHoursPath = "";
    std::string m_rbSteerPowerPath = "";
    std::string m_rbSteerEnergyPath = "";
    std::string m_rbSteerTotalPowerPath = "";
    std::string m_rbSteerWattHoursPath = "";
    std::string m_rbDrivePowerPath = "";
    std::string m_rbDriveEnergyPath = "";
    std::string m_rbDriveTotalPowerPath = "";
    std::string m_rbDriveWattHoursPath = "";
    std::string m_swerveChassisTotalPowerPath = "";
    std::string m_swerveChassisWattHoursPath = "";
    std::string m_headingStatePath = "";
    std::string m_driveStatePath = "";

    DragonDataLoggerSignals();
    virtual ~DragonDataLoggerSignals() = delete;

    static DragonDataLoggerSignals *m_instance;
};
