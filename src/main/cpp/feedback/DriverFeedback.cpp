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

#include "frc/DriverStation.h"
#include "feedback/DriverFeedback.h"
#include "state/RobotState.h"
#include "state/RobotStateChanges.h"
#include "state/IRobotStateChangeSubscriber.h"
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/DriverStation.h>

#include "teleopcontrol/TeleopControl.h"
#include "configs/MechanismConfigMgr.h"
#include "mechanisms/DragonTale/DragonTale.h"
#include "utils/logging/debug/Logger.h"
#include "vision/DragonVision.h"
#include "vision/DragonQuest.h"
#include "chassis/pose/DragonVisionPoseEstimator.h"
#include "chassis/ChassisConfigMgr.h"
#include "vision/definitions/CameraConfigMgr.h"

using frc::DriverStation;

DriverFeedback *DriverFeedback::m_instance = nullptr;

DriverFeedback *DriverFeedback::GetInstance()
{
    if (DriverFeedback::m_instance == nullptr)
    {
        DriverFeedback::m_instance = new DriverFeedback();
    }
    return DriverFeedback::m_instance;
}

void DriverFeedback::UpdateFeedback()
{
    UpdateLEDStates();
    UpdateDiagnosticLEDs();
    CheckControllers();
    m_LEDStates->commitLedData();
}

void DriverFeedback::UpdateRumble()
{
    auto controller = TeleopControl::GetInstance();
    if (!frc::DriverStation::IsTeleop() || m_climbMode)
    {
        controller->SetRumble(0, false, false);
        controller->SetRumble(1, false, false);
    }
    else
    {
        controller->SetRumble(1, m_driveToIsDone, m_driveToIsDone);
    }
}

void DriverFeedback::UpdateLEDStates()
{
    auto mechanismConfigMgr = MechanismConfigMgr::GetInstance()->GetCurrentConfig();
    StateMgr *taleStateManager = mechanismConfigMgr != nullptr ? mechanismConfigMgr->GetMechanism(MechanismTypes::DRAGON_TALE) : nullptr;
    auto taleMgr = taleStateManager != nullptr ? dynamic_cast<DragonTale *>(taleStateManager) : nullptr;
    oldState = currentState;
    if (frc::DriverStation::IsDisabled())
    {
        m_LEDStates->SetChaserPattern(frc::Color::kAqua);
    }
    else
    {
        if (taleMgr != nullptr)
        {
            if (taleMgr->GetRemedialActionStatus())
            {
                m_LEDStates->SetBlinkingPattern(frc::Color::kCrimson, m_blinkingPeriod);
            }
            else if (m_climbMode == RobotStateChanges::ClimbMode::ClimbModeOn)
            {
                currentState = frc::Color::kRed;
                m_LEDStates->SetSolidColor(currentState);
            }
            else if (((m_driveStateType == ChassisOptionEnums::DriveStateType::DRIVE_TO_LEFT_REEF_BRANCH) || (m_driveStateType == ChassisOptionEnums::DriveStateType::DRIVE_TO_RIGHT_REEF_BRANCH) || (m_driveStateType == ChassisOptionEnums::DriveStateType::DRIVE_TO_CORAL_STATION) || (m_driveStateType == ChassisOptionEnums::DriveStateType::DRIVE_TO_BARGE)) && frc::DriverStation::IsAutonomous())
            {
                currentState = frc::Color::kGreen;
                if (m_driveToIsDone)
                {
                    m_LEDStates->SetBlinkingPattern(currentState, m_blinkingPeriod);
                }
                else
                {
                    m_LEDStates->SetSolidColor(currentState);
                }
            }
            else
            {
                if (taleMgr->GetCurrentState() == taleMgr->STATE_GRAB_ALGAE_FLOOR || taleMgr->GetCurrentState() == taleMgr->STATE_GRAB_ALGAE_REEF || taleMgr->GetCurrentState() == taleMgr->STATE_NET || taleMgr->GetCurrentState() == taleMgr->STATE_PROCESS || (taleMgr->GetCurrentState() == taleMgr->STATE_SCORE_ALGAE))
                {
                    currentState = frc::Color::kAqua;
                }
                else if (taleMgr->GetCurrentState() == taleMgr->STATE_READY)
                {
                    currentState = m_scoringMode == RobotStateChanges::ScoringMode::Coral ? frc::Color::kGhostWhite : frc::Color::kAqua;
                }
                else
                {
                    currentState = frc::Color::kGhostWhite;
                }

                if ((taleMgr->GetCurrentState() == taleMgr->STATE_GRAB_ALGAE_REEF) || (taleMgr->GetCurrentState() == taleMgr->STATE_GRAB_ALGAE_FLOOR) || (taleMgr->GetCurrentState() == taleMgr->STATE_HUMAN_PLAYER_LOAD))
                {
                    m_LEDStates->SetBlinkingPattern(currentState, m_blinkingPeriod);
                }
                else if (taleMgr->GetCurrentState() == taleMgr->STATE_HOLD)
                {
                    if (taleMgr->GetCoralOutSensorState() && taleMgr->GetAlgaeSensorState())
                    {
                        m_LEDStates->SetAlternatingColorBlinkingPattern(frc::Color::kGhostWhite, frc::Color::kAqua);
                    }
                    else if (taleMgr->GetCoralOutSensorState() || taleMgr->GetAlgaeSensorState())
                    {
                        m_LEDStates->SetBreathingPattern(currentState, m_breathingPeriod);
                    }
                }
                else if (taleMgr->GetCurrentState() == taleMgr->STATE_L1SCORING_POSITION ||
                         taleMgr->GetCurrentState() == taleMgr->STATE_L2SCORING_POSITION ||
                         taleMgr->GetCurrentState() == taleMgr->STATE_L3SCORING_POSITION ||
                         taleMgr->GetCurrentState() == taleMgr->STATE_L4SCORING_POSITION ||
                         taleMgr->GetCurrentState() == taleMgr->STATE_NET ||
                         taleMgr->GetCurrentState() == taleMgr->STATE_PROCESS)
                {
                    taleMgr->AtTarget() ? m_LEDStates->SetBlinkingPattern(currentState, m_blinkingPeriod) : m_LEDStates->SetSolidColor(currentState); // TODO: add vision alignment to this condition
                }
                else
                {
                    m_LEDStates->SetSolidColor(currentState);
                }
            }
        }
    }
    if (oldState != currentState)
    {
        m_LEDStates->ResetVariables();
    }
}

void DriverFeedback::UpdateDiagnosticLEDs()
{
    bool coralInSensor = false;
    bool coralOutSensor = false;
    bool algaeSensor = false;
    bool questStatus = false;
    bool ll1Status = false;
    if (MechanismConfigMgr::GetInstance()->GetCurrentConfig() != nullptr)
    {
        StateMgr *taleStateManager = MechanismConfigMgr::GetInstance()->GetCurrentConfig()->GetMechanism(MechanismTypes::DRAGON_TALE);
        auto taleMgr = taleStateManager != nullptr ? dynamic_cast<DragonTale *>(taleStateManager) : nullptr;
        if (taleMgr != nullptr)
        {
            coralInSensor = taleMgr->GetCoralInSensorState();
            coralOutSensor = taleMgr->GetCoralOutSensorState();
            algaeSensor = taleMgr->GetAlgaeSensorState();
        }
    }
    auto dragonVision = DragonVision::GetDragonVision();
    if (dragonVision != nullptr)
    {
        auto visionPoseEstitmators = dragonVision->GetPoseEstimators();
        if (!visionPoseEstitmators.empty())
        {
            if (!CameraConfigMgr::GetInstance()->GetCurrentConfig()->GetLimelightIndexs().empty())
            {
                auto limeLight = dynamic_cast<DragonLimelight *>(visionPoseEstitmators[CameraConfigMgr::GetInstance()->GetCurrentConfig()->GetLimelightIndexs()[0]]);
                if (limeLight != nullptr)
                {
                    ll1Status = limeLight->HealthCheck();
                }
            }
            if (CameraConfigMgr::GetInstance()->GetCurrentConfig()->GetQuestIndex() != -1)
            {
                auto quest = dynamic_cast<DragonQuest *>(visionPoseEstitmators[CameraConfigMgr::GetInstance()->GetCurrentConfig()->GetQuestIndex()]);
                if (quest != nullptr)
                {
                    questStatus = quest->HealthCheck();
                }
            }
        }
    }

    m_LEDStates->DiagnosticPattern(FMSData::GetAllianceColor(), coralInSensor, coralOutSensor, algaeSensor, questStatus, ll1Status);
}

void DriverFeedback::ResetRequests(void)
{
}

DriverFeedback::DriverFeedback() : IRobotStateChangeSubscriber()
{

    RobotState *RobotStates = RobotState::GetInstance();
    RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::DesiredScoringMode_Int);
    RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::ClimbModeStatus_Int);
    RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::DriveToFieldElementIsDone_Bool);
    RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::DriveStateType_Int);
}
void DriverFeedback::NotifyStateUpdate(RobotStateChanges::StateChange change, int value)
{
    if (RobotStateChanges::StateChange::ClimbModeStatus_Int == change)
        m_climbMode = static_cast<RobotStateChanges::ClimbMode>(value);

    else if (RobotStateChanges::StateChange::DesiredScoringMode_Int == change)
        m_scoringMode = static_cast<RobotStateChanges::ScoringMode>(value);

    else if (RobotStateChanges::StateChange::DriveStateType_Int == change)
        m_driveStateType = static_cast<ChassisOptionEnums::DriveStateType>(value);
}

void DriverFeedback::NotifyStateUpdate(RobotStateChanges::StateChange change, bool value)
{
    if (RobotStateChanges::StateChange::DriveToFieldElementIsDone_Bool == change)
        m_driveToIsDone = value;
}

void DriverFeedback::CheckControllers()
{
    if (m_controllerCounter == 0)
    {
        auto table = nt::NetworkTableInstance::GetDefault().GetTable("XBOX Controller");
        for (auto i = 0; i < DriverStation::kJoystickPorts; ++i)
        {
            table.get()->PutBoolean(std::string("Controller") + std::to_string(i), DriverStation::GetJoystickIsGamepad(i));
        }
    }
    m_controllerCounter++;
    if (m_controllerCounter > 25)
    {
        m_controllerCounter = 0;
    }
}