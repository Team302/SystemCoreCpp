// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

#include "RobotContainer.h"
#include "RobotIdentifier.h"
#include "auton/AutonPreviewer.h"
#include "auton/CyclePrimitives.h"
#include "auton/drivePrimitives/AutonUtils.h"
#include "chassis/ChassisConfigMgr.h"
#include "configs/MechanismConfig.h"
#include "configs/MechanismConfigMgr.h"
#include "ctre/phoenix6/SignalLogger.hpp"
#include "feedback/DriverFeedback.h"
#include "fielddata/BargeHelper.h"
#include "fielddata/ReefHelper.h"
#include "frc/RobotController.h"
#include "frc/Threads.h"
#include "state/RobotState.h"
#include "teleopcontrol/TeleopControl.h"
#include "utils/DragonField.h"
#include "utils/PeriodicLooper.h"
#include "utils/RoboRio.h"
#include "utils/logging/debug/Logger.h"
#include "utils/logging/signals/DragonDataLoggerMgr.h"
#include "vision/DragonQuest.h"
#include "vision/DragonVision.h"
#include "vision/definitions/CameraConfig.h"
#include "vision/definitions/CameraConfigMgr.h"

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
    if (m_cyclePrims != nullptr)
    {
        m_cyclePrims->Run();
    }
    PeriodicLooper::GetInstance()->AutonRunCurrentState();
}

void Robot::TeleopInit()
{
    PeriodicLooper::GetInstance()->TeleopRunCurrentState();
}

void Robot::TeleopPeriodic()
{
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

    new RobotContainer(); // instantiate RobotContainer to setup commands and subsystems

    MechanismConfigMgr::GetInstance()->InitRobot((RobotIdentifier)teamNumber);

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

    auto chassis = ChassisConfigMgr::GetInstance()->GetSwerveChassis();
    if (m_field != nullptr && chassis != nullptr)
    {
        m_field->UpdateRobotPosition(chassis->GetPose());
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
