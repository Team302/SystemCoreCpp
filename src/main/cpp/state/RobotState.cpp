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
//======================================================\==============================================================================================

#include "state/RobotState.h"

#include <string>
#include <vector>

#include "chassis/ChassisConfigMgr.h"
#include "state/RobotStateChangeBroker.h"
#include "teleopcontrol/TeleopControl.h"
#include "utils/DragonField.h"
#include "mechanisms/DragonTale/DragonTale.h"
#include "state/StateMgr.h"

using frc::DriverStation;

RobotState *RobotState::m_instance = nullptr;

RobotState *RobotState::GetInstance()
{
    if (RobotState::m_instance == nullptr)
    {
        RobotState::m_instance = new RobotState();
    }
    return RobotState::m_instance;
}

RobotState::RobotState() : m_brokers(),
                           m_scoringMode(RobotStateChanges::ScoringMode::Coral),
                           m_gamePhase(RobotStateChanges::Disabled),
                           m_climbMode(RobotStateChanges::ClimbModeOff),
                           m_scoringModeButtonReleased(true)
{
    m_brokers.reserve(RobotStateChanges::LoopCounterStart);
    auto start = static_cast<int>(RobotStateChanges::LoopCounterStart);
    auto end = static_cast<int>(RobotStateChanges::LoopCounterEnd);
    for (auto i = start; i < end; ++i)
    {
        m_brokers.emplace_back(new RobotStateChangeBroker(static_cast<RobotStateChanges::StateChange>(i)));
    }
}

RobotState::~RobotState()
{
    for (auto broker : m_brokers)
    {
        delete broker;
    }
    m_brokers.clear();
}

void RobotState::Init()
{
}

void RobotState::Run()
{
    PublishGameStateChanges();
    if (DriverStation::IsTeleopEnabled())
    {
        auto controller = TeleopControl::GetInstance();
        if (controller != nullptr)
        {
            PublishScoringMode(controller);
            PublishClimbMode(controller);
            PublishDesiredCoralSide(controller);
        }
    }
}

void RobotState::RegisterForStateChanges(IRobotStateChangeSubscriber *subscriber, RobotStateChanges::StateChange change)
{
    auto slot = static_cast<unsigned int>(change);
    if (slot < m_brokers.size())
    {
        m_brokers[slot]->AddSubscriber(subscriber);
    }
}

void RobotState::PublishStateChange(RobotStateChanges::StateChange change, int newValue)
{
    auto slot = static_cast<unsigned int>(change);
    if (slot < m_brokers.size())
    {
        m_brokers[slot]->Notify(newValue);
    }
}

void RobotState::PublishStateChange(RobotStateChanges::StateChange change, double newValue)
{
    auto slot = static_cast<unsigned int>(change);
    if (slot < m_brokers.size())
    {
        m_brokers[slot]->Notify(newValue);
    }
}
void RobotState::PublishStateChange(RobotStateChanges::StateChange change, units::length::meter_t newValue)
{
    auto slot = static_cast<unsigned int>(change);
    if (slot < m_brokers.size())
    {
        m_brokers[slot]->Notify(newValue);
    }
}
void RobotState::PublishStateChange(RobotStateChanges::StateChange change, units::angle::degree_t newValue)
{
    auto slot = static_cast<unsigned int>(change);
    if (slot < m_brokers.size())
    {
        m_brokers[slot]->Notify(newValue);
    }
}
void RobotState::PublishStateChange(RobotStateChanges::StateChange change, units::velocity::meters_per_second_t newValue)
{
    auto slot = static_cast<unsigned int>(change);
    if (slot < m_brokers.size())
    {
        m_brokers[slot]->Notify(newValue);
    }
}
void RobotState::PublishStateChange(RobotStateChanges::StateChange change, units::angular_velocity::degrees_per_second_t newValue)
{
    auto slot = static_cast<unsigned int>(change);
    if (slot < m_brokers.size())
    {
        m_brokers[slot]->Notify(newValue);
    }
}
void RobotState::PublishStateChange(RobotStateChanges::StateChange change, frc::Pose2d newValue)
{
    auto slot = static_cast<unsigned int>(change);
    if (slot < m_brokers.size())
    {
        m_brokers[slot]->Notify(newValue);
    }
}
void RobotState::PublishStateChange(RobotStateChanges::StateChange change, bool newValue)
{
    auto slot = static_cast<unsigned int>(change);
    if (slot < m_brokers.size())
    {
        m_brokers[slot]->Notify(newValue);
    }
}

void RobotState::PublishGameStateChanges()
{
    auto gameState = m_gamePhase;
    if (frc::DriverStation::IsEnabled())
    {
        if (DriverStation::IsAutonomousEnabled())
        {
            gameState = RobotStateChanges::Auton;
        }
        else if (DriverStation::IsTeleopEnabled())
        {
            gameState = RobotStateChanges::Teleop;
        }
    }
    else
    {
        gameState = RobotStateChanges::Disabled;
    }

    if (gameState != m_gamePhase)
    {
        m_gamePhase = gameState;
        PublishStateChange(RobotStateChanges::GameState_Int, gameState);
    }
}
void RobotState::PublishScoringMode(TeleopControl *controller)
{
    if (controller->IsButtonPressed(TeleopControlFunctions::SCORING_MODE))
    {
        if (m_scoringModeButtonReleased)
        {
            m_scoringMode = (m_scoringMode == RobotStateChanges::ScoringMode::Coral) ? RobotStateChanges::ScoringMode::Algae : RobotStateChanges::ScoringMode::Coral;
            PublishStateChange(RobotStateChanges::StateChange::DesiredScoringMode_Int, m_scoringMode);
        }
    }
    else if (MechanismConfigMgr::GetInstance()->GetCurrentConfig() != nullptr)
    {

        if (MechanismConfigMgr::GetInstance()->GetCurrentConfig()->GetMechanism(MechanismTypes::DRAGON_TALE) != nullptr)
        {
            if (MechanismConfigMgr::GetInstance()->GetCurrentConfig()->GetMechanism(MechanismTypes::DRAGON_TALE)->GetCurrentState() == DragonTale::STATE_NAMES::STATE_GRAB_ALGAE_REEF)
            {
                m_scoringMode = RobotStateChanges::Algae;
            }
        }
    }
    m_scoringModeButtonReleased = !controller->IsButtonPressed(TeleopControlFunctions::SCORING_MODE);
}

void RobotState::PublishClimbMode(TeleopControl *controller)
{
    if (controller->IsButtonPressed(TeleopControlFunctions::CLIMB_MODE))
    {
        if (m_climbModeButtonReleased)
        {
            m_climbMode = (m_climbMode == RobotStateChanges::ClimbModeOff) ? RobotStateChanges::ClimbModeOn : RobotStateChanges::ClimbModeOff;

            PublishStateChange(RobotStateChanges::ClimbModeStatus_Int, m_climbMode);
        }
    }

    m_climbModeButtonReleased = !controller->IsButtonPressed(TeleopControlFunctions::CLIMB_MODE);
}

void RobotState::PublishDesiredCoralSide(TeleopControl *controller)
{
    if (controller->IsButtonPressed(TeleopControlFunctions::SWITCH_DESIRED_CORAL_SIDE))
    {
        if (m_desiredCoralSideButtonReleased)
        {
            m_desiredCoralSide = (m_desiredCoralSide == RobotStateChanges::DesiredCoralSide::AllianceWall) ? RobotStateChanges::DesiredCoralSide::Sidewall : RobotStateChanges::DesiredCoralSide::AllianceWall;
            PublishStateChange(RobotStateChanges::DesiredCoralSide_Int, m_desiredCoralSide);
        }
    }

    m_desiredCoralSideButtonReleased = !controller->IsButtonPressed(TeleopControlFunctions::SWITCH_DESIRED_CORAL_SIDE);
}