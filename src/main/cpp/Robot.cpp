// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

#include "auton/AutonPreviewer.h"
#include "auton/CyclePrimitives.h"
#include "auton/drivePrimitives/AutonUtils.h"
#include "chassis/ChassisConfigMgr.h"
#include "chassis/pose/DragonSwervePoseEstimator.h"
#include "configs/MechanismConfig.h"
#include "configs/MechanismConfigMgr.h"
#include "ctre/phoenix6/SignalLogger.hpp"
#include "feedback/DriverFeedback.h"
#include "fielddata/BargeHelper.h"
#include "fielddata/BargeHelper.h"
#include "fielddata/ReefHelper.h"
#include "fielddata/ReefHelper.h"
#include "frc/RobotController.h"
#include "frc/Threads.h"
#include "RobotIdentifier.h"
#include "state/RobotState.h"
#include "teleopcontrol/TeleopControl.h"
#include "utils/DragonField.h"
#include "utils/logging/debug/Logger.h"
#include "utils/logging/signals/DragonDataLoggerMgr.h"
#include "utils/PeriodicLooper.h"
#include "utils/RoboRio.h"
#include "utils/sensors/SensorData.h"
#include "utils/sensors/SensorDataMgr.h"
#include "vision/definitions/CameraConfig.h"
#include "vision/definitions/CameraConfigMgr.h"
#include "vision/DragonVision.h"
#include "vision/DragonQuest.h"
#include "chassis/SwerveContainer.h"

Robot::Robot()
{
    Logger::GetLogger()->PutLoggingSelectionsOnDashboard();

    InitializeRobot();
    InitializeAutonOptions();
    InitializeDriveteamFeedback();

    BargeHelper::GetInstance();
    ReefHelper::GetInstance();

    m_datalogger = DragonDataLoggerMgr::GetInstance();

    auto path = AutonUtils::GetTrajectoryFromPathFile("BlueLeftInside_I"); // load choreo library so we don't get loop overruns during autonperiodic
}

void Robot::RobotPeriodic()
{
    frc2::CommandScheduler::GetInstance().Run();

    isFMSAttached = frc::DriverStation::IsFMSAttached();
    if (!isFMSAttached)
    {
        Logger::GetLogger()->PeriodicLog();
    }

    // if (m_datalogger != nullptr && !frc::DriverStation::IsDisabled())
    // {
    //     m_datalogger->PeriodicDataLog();
    // }

    if (m_robotState != nullptr)
    {
        m_robotState->Run();
    }

    if (m_quest != nullptr)
    {
        m_quest->HandleHeartBeat();
        m_quest->RefreshNT();
    }

    UpdateDriveTeamFeedback();
}

void Robot::DisabledPeriodic()
{
    if (m_dragonswerveposeestimator != nullptr)
    {
        m_dragonswerveposeestimator->CalculateInitialPose();
    }
}

void Robot::AutonomousInit()
{
    frc::SetCurrentThreadPriority(true, 15);

    if (m_cyclePrims != nullptr)
    {
        m_cyclePrims->Init();
    }
    PeriodicLooper::GetInstance()->AutonRunCurrentState();
}

void Robot::AutonomousPeriodic()
{
    SensorDataMgr::GetInstance()->CacheData();
    if (m_dragonswerveposeestimator != nullptr)
    {
        m_dragonswerveposeestimator->Update();
    }

    if (m_cyclePrims != nullptr)
    {
        m_cyclePrims->Run();
    }
    PeriodicLooper::GetInstance()->AutonRunCurrentState();
}

void Robot::TeleopInit()
{
    PeriodicLooper::GetInstance()->TeleopRunCurrentState();
    frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TeleopPeriodic()
{
    SensorDataMgr::GetInstance()->CacheData();
    if (m_dragonswerveposeestimator != nullptr)
    {
        m_dragonswerveposeestimator->Update();
    }
    PeriodicLooper::GetInstance()->TeleopRunCurrentState();
}

void Robot::TestInit()
{
    frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::InitializeRobot()
{
    int32_t teamNumber = frc::RobotController::GetTeamNumber();
    FieldConstants::GetInstance();
    RoboRio::GetInstance();
    auto chassisConfig = ChassisConfigMgr::GetInstance();
    chassisConfig->CreateDrivetrain();
    m_container = SwerveContainer::GetInstance();

    MechanismConfigMgr::GetInstance()->InitRobot((RobotIdentifier)teamNumber);

    CameraConfigMgr::GetInstance()->InitCameras(static_cast<RobotIdentifier>(teamNumber));

    m_dragonswerveposeestimator = DragonSwervePoseEstimator::GetInstance();

    auto dragonVision = DragonVision::GetDragonVision();
    if (dragonVision != nullptr)
    {
        auto visionPoseEstimators = dragonVision->GetPoseEstimators();
        for (auto &poseEstimator : visionPoseEstimators)
        {
            m_dragonswerveposeestimator->RegisterVisionPoseEstimator(poseEstimator);
        }
        if (!visionPoseEstimators.empty())
        {
            if (CameraConfigMgr::GetInstance()->GetCurrentConfig()->GetQuestIndex() != -1)
            {
                m_quest = static_cast<DragonQuest *>(visionPoseEstimators[CameraConfigMgr::GetInstance()->GetCurrentConfig()->GetQuestIndex()]);
            }
        }
    }

    m_robotState = RobotState::GetInstance();
    m_robotState->Init();
}

void Robot::InitializeAutonOptions()
{
    m_cyclePrims = new CyclePrimitives(); // intialize auton selections
    m_previewer = new AutonPreviewer(m_cyclePrims);
}
void Robot::InitializeDriveteamFeedback()
{
    m_field = DragonField::GetInstance(); // TODO: move to drive team feedback
}

void Robot::UpdateDriveTeamFeedback()
{
    if (m_previewer != nullptr)
    {
        m_previewer->CheckCurrentAuton();
    }
    if (m_field != nullptr && m_dragonswerveposeestimator != nullptr)
    {
        m_field->UpdateRobotPosition(m_dragonswerveposeestimator->GetPose());
    }
    auto feedback = DriverFeedback::GetInstance();
    if (feedback != nullptr)
    {
        feedback->UpdateFeedback();
    }
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
