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

#include <string>
#include <vector>

#include "frc/smartdashboard/SendableChooser.h"
#include "frc/smartdashboard/SmartDashboard.h"

#include "networktables/DoubleArrayTopic.h"
#include "networktables/DoubleTopic.h"
#include "networktables/IntegerTopic.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

#include "state/IRobotStateChangeSubscriber.h"
#include "utils/logging/signals/DragonDataLogger.h"
#include "vision/DragonVisionPoseEstimatorStruct.h"

using namespace std;

class DragonQuest : public IRobotStateChangeSubscriber, public DragonDataLogger

{
public:
    DragonQuest(
        units::length::inch_t mountingXOffset, /// <I> x offset of Quest from robot center (forward relative to robot)
        units::length::inch_t mountingYOffset, /// <I> y offset of Quest from robot center (left relative to robot)
        units::length::inch_t mountingZOffset, /// <I> z offset of Quest from robot center (up relative to robot)
        units::angle::degree_t mountingPitch,  /// <I> - Pitch of Quest
        units::angle::degree_t mountingYaw,    /// <I> - Yaw of Quest
        units::angle::degree_t mountingRoll    /// <I> - Roll of Quest
    );
    frc::Pose2d GetEstimatedPose();
    void DataLog(uint64_t timestamp) override;
    bool HealthCheck() { return m_isConnected; };
    void SetIsConnected();

    void SetRobotPose(const frc::Pose2d &pose);
    DragonVisionPoseEstimatorStruct GetPoseEstimate();

    void RefreshNT();
    void HandleHeartBeat();

    void HandleDashboard();

    void NotifyStateUpdate(RobotStateChanges::StateChange change, int value) override;

private:
    DragonQuest() = delete;
    void ZeroPosition();

    units::length::inch_t m_mountingXOffset; /// <I> x offset of Quest from robot center (forward relative to robot)
    units::length::inch_t m_mountingYOffset; /// <I> y offset of Quest from robot center (left relative to robot)
    units::length::inch_t m_mountingZOffset; /// <I> z offset of Quest from robot center (up relative to robot)
    units::angle::degree_t m_mountingPitch;  /// <I> - Pitch of Quest
    units::angle::degree_t m_mountingYaw;    /// <I> - Yaw of Quest
    units::angle::degree_t m_mountingRoll;   /// <I> - Roll of Quest

    std::shared_ptr<nt::NetworkTable> m_networktable;
    static DragonQuest *m_dragonquest;
    nt::IntegerSubscriber m_questMiso;
    nt::IntegerPublisher m_questMosi;
    nt::DoubleArrayTopic m_posTopic;
    nt::DoubleArrayTopic m_rotationTopic;
    nt::IntegerTopic m_frameCountTopic;
    nt::DoubleArrayPublisher m_initialPosePublisher;
    nt::DoubleSubscriber m_heartbeatRequestSub;
    nt::DoublePublisher m_heartbeatResponsePub;
    nt::DoubleSubscriber m_timestamp;

    frc::SendableChooser<bool> m_questEnabledChooser;
    frc::SendableChooser<bool> m_questEndgameEnabledChooser;

    bool m_hasReset = false;
    bool m_isConnected = false;

    frc::Transform2d m_questToRobotTransform; // <I> Transform from Quest to robot (used to calculate the robot pose from the quest pose)

    static constexpr double m_stdxy{0.02};
    static constexpr double m_stddeg{.035};

    double m_prevFrameCount = 0;
    int m_loopCounter = 0;

    int m_lastProcessedHeartbeatId = 0;

    frc::Pose2d m_rawQuestPose;

    bool m_isQuestEnabled = false; // <I> Is the Quest enabled?
    RobotStateChanges::ClimbMode m_climbMode = RobotStateChanges::ClimbMode::ClimbModeOff;
};