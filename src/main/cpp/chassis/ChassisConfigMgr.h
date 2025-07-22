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

#include "chassis/generated/TunerConstants302.h"
#include "chassis/generated/TunerConstants9998.h"
#include "chassis/generated/CommandSwerveDrivetrain.h"

using namespace ctre::phoenix6;

namespace subsystems
{
    class CommandSwerveDrivetrain; // Forward declaration
}

class ChassisConfigMgr
{
public:
    static ChassisConfigMgr *GetInstance();

    void CreateDrivetrain();

    subsystems::CommandSwerveDrivetrain *GetSwerveChassis();

    units::meters_per_second_t GetMaxSpeed() { return m_maxSpeed; }
    double GetRotationRateDegreesPerSecond() const { return m_chassis != nullptr ? m_chassis.get()->GetPigeon2().GetAngularVelocityZWorld(true).GetValueAsDouble() : 0.0; }

private:
    ChassisConfigMgr();
    virtual ~ChassisConfigMgr() = default;

    static ChassisConfigMgr *m_instance;
    units::meters_per_second_t m_maxSpeed;
    std::unique_ptr<subsystems::CommandSwerveDrivetrain> m_chassis;
};
