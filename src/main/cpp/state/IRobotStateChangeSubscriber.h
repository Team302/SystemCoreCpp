
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

#include <state/RobotStateChanges.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <frc/geometry/Pose2d.h>

class IRobotStateChangeSubscriber
{
public:
    IRobotStateChangeSubscriber() = default;
    ~IRobotStateChangeSubscriber() = default;

    virtual void NotifyStateUpdate(RobotStateChanges::StateChange change, int value) {}
    virtual void NotifyStateUpdate(RobotStateChanges::StateChange change, double value) {}
    virtual void NotifyStateUpdate(RobotStateChanges::StateChange change, units::length::meter_t value) {}
    virtual void NotifyStateUpdate(RobotStateChanges::StateChange change, units::angle::degree_t value) {}
    virtual void NotifyStateUpdate(RobotStateChanges::StateChange change, units::velocity::meters_per_second_t value) {}
    virtual void NotifyStateUpdate(RobotStateChanges::StateChange change, units::angular_velocity::degrees_per_second_t value) {}
    virtual void NotifyStateUpdate(RobotStateChanges::StateChange change, frc::Pose2d value) {}
    virtual void NotifyStateUpdate(RobotStateChanges::StateChange change, bool value) {}
};
