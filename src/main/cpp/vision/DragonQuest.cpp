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
#include "units/time.h"
#include "utils/AngleUtils.h"
#include "utils/DragonField.h"
#include "vision/DragonQuest.h"
#include "state/RobotStateChanges.h"
#include "state/RobotState.h"
#include "state/IRobotStateChangeSubscriber.h"
#include "utils/logging/debug/Logger.h"

DragonQuest::DragonQuest(
    units::length::inch_t mountingXOffset, /// <I> x offset of Quest from robot center (forward relative to robot)
    units::length::inch_t mountingYOffset, /// <I> y offset of Quest from robot center (left relative to robot)
    units::length::inch_t mountingZOffset, /// <I> z offset of Quest from robot center (up relative to robot)
    units::angle::degree_t mountingPitch,  /// <I> - Pitch of Quest
    units::angle::degree_t mountingYaw,    /// <I> - Yaw of Quest
    units::angle::degree_t mountingRoll    /// <I> - Roll of Quest
    ) : IRobotStateChangeSubscriber(),
        m_mountingXOffset(mountingXOffset),
        m_mountingYOffset(mountingYOffset),
        m_mountingZOffset(mountingZOffset),
        m_mountingPitch(mountingPitch),
        m_mountingYaw(mountingYaw),
        m_mountingRoll(mountingRoll)
{
    m_networktable = nt::NetworkTableInstance::GetDefault().GetTable(std::string("questnav"));
    m_questMosi = m_networktable.get()->GetIntegerTopic("mosi").Publish();
    m_questMiso = m_networktable.get()->GetIntegerTopic("miso").Subscribe(0);
    m_posTopic = m_networktable.get()->GetDoubleArrayTopic("position");
    m_frameCountTopic = m_networktable.get()->GetIntegerTopic("frameCount");

    m_rotationTopic = m_networktable.get()->GetDoubleArrayTopic("euler angles");
    m_initialPosePublisher = m_networktable.get()->GetDoubleArrayTopic("resetpose").Publish();

    m_heartbeatRequestSub = m_networktable.get()->GetDoubleTopic("heartbeat/quest_to_robot").Subscribe(0);
    m_heartbeatResponsePub = m_networktable.get()->GetDoubleTopic("heartbeat/robot_to_quest").Publish();

    m_timestamp = m_networktable.get()->GetDoubleTopic("timestamp").Subscribe(0);

    m_questToRobotTransform = frc::Transform2d{
        frc::Translation2d(m_mountingXOffset, m_mountingYOffset),
        frc::Rotation2d(m_mountingYaw)};

    m_questMosi.Set(0); // initial idle state

    m_questEnabledChooser.AddOption("ON", true);
    m_questEnabledChooser.AddOption("OFF", false);
    m_questEnabledChooser.SetDefaultOption("ON", true);

    m_questEndgameEnabledChooser.AddOption("ENDGAME ONLY", true);
    m_questEndgameEnabledChooser.AddOption("FULL MATCH", false);
    m_questEndgameEnabledChooser.SetDefaultOption("ENDGAME ONLY", true);

    frc::SmartDashboard::PutData("Quest ON/OFF", &m_questEnabledChooser);
    frc::SmartDashboard::PutData("Quest Endgame ONLY", &m_questEndgameEnabledChooser);
    RobotState *RobotStates = RobotState::GetInstance();
    RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::ClimbModeStatus_Int);
}

frc::Pose2d DragonQuest::GetEstimatedPose()
{

    std::vector<double> posarray = m_posTopic.GetEntry(std::array<double, 3>{}).Get();
    std::vector<double> rotationarray = m_rotationTopic.GetEntry(std::array<double, 3>{}).Get();

    units::length::meter_t x{posarray[2]};
    units::length::meter_t y{-posarray[0]};
    units::angle::degree_t yaw{-rotationarray[1]};

    frc::Pose2d questPose{x, y, yaw};

    frc::Pose2d robotPose = questPose.TransformBy(m_questToRobotTransform.Inverse());

    return robotPose;
}

void DragonQuest::SetIsConnected()
{
    double currentFrameCount = m_frameCountTopic.GetEntry(0).Get();
    if (m_loopCounter > 3)
    {
        if (currentFrameCount != m_prevFrameCount)
        {
            m_loopCounter = 0;
            m_isConnected = true;
        }
        else
        {
            m_isConnected = false;
        }
        m_prevFrameCount = currentFrameCount;
    }

    m_loopCounter = (m_loopCounter > 3) ? 0 : m_loopCounter + 1;
}

void DragonQuest::ZeroPosition()
{
    if (m_questMiso.Get() != 99)
    {
        m_questMosi.Set(1);
    }
}

void DragonQuest::DataLog(uint64_t timestamp)
{
    Log2DPoseData(timestamp, DragonDataLogger::PoseSingals::CURRENT_CHASSIS_QUEST_POSE2D, GetEstimatedPose());
}

void DragonQuest::RefreshNT()
{
    m_posTopic = m_networktable.get()->GetDoubleArrayTopic("position");
    m_rotationTopic = m_networktable.get()->GetDoubleArrayTopic("eulerAngles");
    m_frameCountTopic = m_networktable.get()->GetIntegerTopic("frameCount");
}

void DragonQuest::HandleHeartBeat()
{
    if (m_questMiso.Get() == 98)
    {
        m_questMosi.Set(0);
    }
    double requestId = m_heartbeatRequestSub.Get();
    // Only respond to new requests to avoid flooding
    if (requestId > 0 && requestId != m_lastProcessedHeartbeatId)
    {
        m_heartbeatResponsePub.Set(requestId);
        m_lastProcessedHeartbeatId = requestId;
    }
    SetIsConnected();
}

void DragonQuest::SetRobotPose(const frc::Pose2d &pose)
{
    if (!m_hasReset)
    {
        frc::Pose2d questPose = pose.TransformBy(m_questToRobotTransform);
        m_initialPosePublisher.Set(std::array<double, 3>{questPose.X().value(), questPose.Y().value(), questPose.Rotation().Degrees().value()});

        if (m_questMiso.Get() != 99)
        {
#ifndef _WIN32
            sleep(1);
#endif
            m_questMosi.Set(2);
        }
        m_hasReset = true;
    }
}

void DragonQuest::HandleDashboard()
{

    if (m_questEnabledChooser.GetSelected() == true)
    {
        m_isQuestEnabled = true;
        if (m_questEndgameEnabledChooser.GetSelected() == true && m_climbMode != RobotStateChanges::ClimbMode::ClimbModeOn)
        {
            m_isQuestEnabled = false;
        }
    }
    else
    {
        m_isQuestEnabled = false;
    }
}

void DragonQuest::NotifyStateUpdate(RobotStateChanges::StateChange change, int value)
{
    if (RobotStateChanges::StateChange::ClimbModeStatus_Int == change)
        m_climbMode = static_cast<RobotStateChanges::ClimbMode>(value);
}
DragonVisionPoseEstimatorStruct DragonQuest::GetPoseEstimate()
{
    DragonVisionPoseEstimatorStruct str;
    HandleDashboard();
    if (!m_hasReset || !m_isConnected || !m_isQuestEnabled)
    {
        str.m_confidenceLevel = DragonVisionPoseEstimatorStruct::ConfidenceLevel::NONE;
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("questnav"), string("confidence"), string("NONE"));
    }
    else
    {
        str.m_confidenceLevel = DragonVisionPoseEstimatorStruct::ConfidenceLevel::HIGH;
        str.m_visionPose = GetEstimatedPose();
        str.m_stds = wpi::array{m_stdxy, m_stdxy, m_stddeg};
        str.m_timeStamp = units::time::second_t(m_timestamp.GetAtomic().serverTime);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("questnav"), string("confidence"), string("HIGH"));
    }
    return str;
}
