
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

#include <algorithm>
#include <vector>
#include <state/RobotStateChanges.h>
#include <state/RobotStateChangeBroker.h>
#include <state/IRobotStateChangeSubscriber.h>

RobotStateChangeBroker::RobotStateChangeBroker(RobotStateChanges::StateChange change) : m_change(change),
                                                                                        m_subscribers()
{
}

void RobotStateChangeBroker::AddSubscriber(IRobotStateChangeSubscriber *item)
{
    if (std::find(m_subscribers.begin(), m_subscribers.end(), item) == m_subscribers.end())
    {
        m_subscribers.emplace_back(item);
    }
}

void RobotStateChangeBroker::Notify(int value)
{
    for (auto subscriber : m_subscribers)
    {
        subscriber->NotifyStateUpdate(m_change, value);
    }
}
void RobotStateChangeBroker::Notify(double value)
{
    for (auto subscriber : m_subscribers)
    {
        subscriber->NotifyStateUpdate(m_change, value);
    }
}
void RobotStateChangeBroker::Notify(units::length::meter_t value)
{
    for (auto subscriber : m_subscribers)
    {
        subscriber->NotifyStateUpdate(m_change, value);
    }
}
void RobotStateChangeBroker::Notify(units::angle::degree_t value)
{
    for (auto subscriber : m_subscribers)
    {
        subscriber->NotifyStateUpdate(m_change, value);
    }
}
void RobotStateChangeBroker::Notify(units::velocity::meters_per_second_t value)
{
    for (auto subscriber : m_subscribers)
    {
        subscriber->NotifyStateUpdate(m_change, value);
    }
}
void RobotStateChangeBroker::Notify(units::angular_velocity::degrees_per_second_t value)
{
    for (auto subscriber : m_subscribers)
    {
        subscriber->NotifyStateUpdate(m_change, value);
    }
}
void RobotStateChangeBroker::Notify(frc::Pose2d value)
{
    for (auto subscriber : m_subscribers)
    {
        subscriber->NotifyStateUpdate(m_change, value);
    }
}
void RobotStateChangeBroker::Notify(bool value)
{
    for (auto subscriber : m_subscribers)
    {
        subscriber->NotifyStateUpdate(m_change, value);
    }
}
