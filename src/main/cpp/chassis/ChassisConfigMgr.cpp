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

#include "frc/RobotController.h"
#include "chassis/ChassisConfigMgr.h"
#include "units/velocity.h"
#include "chassis/generated/CommandSwerveDrivetrain.h"

#include "chassis/generated/TunerConstants302.h"
#include "chassis/generated/TunerConstants9998.h"
#include "RobotIdentifier.h"

ChassisConfigMgr *ChassisConfigMgr::m_instance = nullptr;

ChassisConfigMgr *ChassisConfigMgr::GetInstance()
{
    if (ChassisConfigMgr::m_instance == nullptr)
    {
        ChassisConfigMgr::m_instance = new ChassisConfigMgr();
    }
    return ChassisConfigMgr::m_instance;
}
ChassisConfigMgr::ChassisConfigMgr() : m_maxSpeed(0_mps)
{
    CreateDrivetrain();
}

void ChassisConfigMgr::CreateDrivetrain()
{
    int32_t teamNumber = frc::RobotController::GetTeamNumber();

    auto id = static_cast<RobotIdentifier>(teamNumber);

    switch (id)
    {
    case RobotIdentifier::COMP_BOT_302:
        m_maxSpeed = TunerConstants302::kSpeedAt12Volts;
        m_chassis = TunerConstants302::CreateDrivetrain(); // This returns a unique_ptr
        break;

    case RobotIdentifier::CHASSIS_BOT_9998:
        m_maxSpeed = TunerConstants9998::kSpeedAt12Volts;
        m_chassis = TunerConstants9998::CreateDrivetrain();
        break;

    case RobotIdentifier::SIM_BOT_0:
        m_maxSpeed = TunerConstants302::kSpeedAt12Volts;
        m_chassis = TunerConstants302::CreateDrivetrain();
        break;

    default:
        // Create a default or log an error if you want
        m_maxSpeed = 0_mps;
        m_chassis = nullptr;
        break;
    }
}
subsystems::CommandSwerveDrivetrain *ChassisConfigMgr::GetSwerveChassis()
{
    if (m_chassis.get() == nullptr)
    {
        CreateDrivetrain();
    }
    return m_chassis.get();
}
