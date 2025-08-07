// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <string>

#include <Robot.h>

#include "auton/AutonPreviewer.h"
#include "auton/CyclePrimitives.h"
#include "auton/drivePrimitives/AutonUtils.h"
#include "chassis/definitions/ChassisConfig.h"
#include "chassis/definitions/ChassisConfigMgr.h"
#include "chassis/HolonomicDrive.h"
#include "chassis/pose/DragonSwervePoseEstimator.h"
#include "chassis/SwerveChassis.h"
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

using std::string;

void Robot::RobotInit()
{
    isFMSAttached = frc::DriverStation::IsFMSAttached();

    Logger::GetLogger()->PutLoggingSelectionsOnDashboard();

    m_controller = nullptr;

    InitializeRobot();
    InitializeAutonOptions();
    InitializeDriveteamFeedback();

    BargeHelper::GetInstance();
    ReefHelper::GetInstance();

    m_datalogger = DragonDataLoggerMgr::GetInstance();

    auto path = AutonUtils::GetPathFromTrajectory("BlueLeftInside_I"); // load choreo library so we don't get loop overruns during autonperiodic

    if (m_dragonswerveposeestimator != nullptr)
    {
        auto visionPoseEstitmators = m_dragonswerveposeestimator->GetVisionPoseEstimators();
        if (!visionPoseEstitmators.empty())
        {
            if (CameraConfigMgr::GetInstance()->GetCurrentConfig()->GetQuestIndex() != -1)
            {
                m_quest = static_cast<DragonQuest *>(visionPoseEstitmators[CameraConfigMgr::GetInstance()->GetCurrentConfig()->GetQuestIndex()]);
            }
        }
    }
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
    isFMSAttached = isFMSAttached ? true : frc::DriverStation::IsFMSAttached();
    if (!isFMSAttached)
    {
        Logger::GetLogger()->PeriodicLog();
    }

    if (m_datalogger != nullptr && !frc::DriverStation::IsDisabled())
    {
        m_datalogger->PeriodicDataLog();
    }

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
/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit()
{
    frc::SetCurrentThreadPriority(true, 15);

    if (m_cyclePrims != nullptr)
    {
        m_cyclePrims->Init();
    }
    PeriodicLooper::GetInstance()->AutonRunCurrentState();
    RobotState::GetInstance()->PublishStateChange(RobotStateChanges::GameState_Int, RobotStateChanges::GamePeriod::Auton);
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
    if (m_controller == nullptr)
    {
        m_controller = TeleopControl::GetInstance();
    }

    if (m_chassis != nullptr && m_controller != nullptr && m_holonomic != nullptr)
    {
        m_holonomic->Init();
    }

    PeriodicLooper::GetInstance()->TeleopRunCurrentState();
    RobotState::GetInstance()->PublishStateChange(RobotStateChanges::GameState_Int, RobotStateChanges::GamePeriod::Teleop);
}

void Robot::TeleopPeriodic()
{
    SensorDataMgr::GetInstance()->CacheData();
    if (m_dragonswerveposeestimator != nullptr)
    {
        m_dragonswerveposeestimator->Update();
    }

    if (m_chassis != nullptr && m_controller != nullptr && m_holonomic != nullptr)
    {
        m_holonomic->Run();
    }
    PeriodicLooper::GetInstance()->TeleopRunCurrentState();
}

void Robot::DisabledInit()
{
    RobotState::GetInstance()->PublishStateChange(RobotStateChanges::GameState_Int, RobotStateChanges::GamePeriod::Disabled);
}

void Robot::DisabledPeriodic()
{
    if (m_dragonswerveposeestimator != nullptr)
    {
        m_dragonswerveposeestimator->CalculateInitialPose();
    }
}

void Robot::TestInit()
{
}

void Robot::TestPeriodic()
{
}

void Robot::SimulationInit()
{
    PeriodicLooper::GetInstance()->SimulationRunCurrentState();
}

void Robot::SimulationPeriodic()
{
    PeriodicLooper::GetInstance()->SimulationRunCurrentState();
}

void Robot::InitializeRobot()
{
    int32_t teamNumber = frc::RobotController::GetTeamNumber();
    FieldConstants::GetInstance();
    RoboRio::GetInstance();

    ChassisConfigMgr::GetInstance()->InitChassis(static_cast<RobotIdentifier>(teamNumber));
    auto chassisConfig = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = chassisConfig != nullptr ? chassisConfig->GetSwerveChassis() : nullptr;
    m_holonomic = nullptr;
    m_dragonswerveposeestimator = nullptr;
    if (m_chassis != nullptr)
    {
        m_holonomic = new HolonomicDrive();
        m_dragonswerveposeestimator = m_chassis->GetSwervePoseEstimator();
    }
    MechanismConfigMgr::GetInstance()->InitRobot((RobotIdentifier)teamNumber);

    // initialize cameras
    CameraConfigMgr::GetInstance()->InitCameras(static_cast<RobotIdentifier>(teamNumber));
    // auto vision = DragonVision::GetDragonVision();

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
        m_field->UpdateRobotPosition(m_dragonswerveposeestimator->GetPose()); // ToDo:: Move to DriveTeamFeedback (also don't assume m_field isn't a nullptr)
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
