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

/*
  Robot.cpp — Developer Notes

  Overview
  - Implements the top-level robot entry point and lifecycle handlers (init/periodic for Disabled/Auton/Teleop/Test).
  - Wires up RobotContainer, subsystems, command scheduling, auton selection, and drive-team feedback.
  - Lightweight: avoid adding heavy computation in periodic methods; use PeriodicLooper for mode-specific loops.

  Key responsibilities
  - Global initialization (logger, configs, chassis, RobotContainer, RobotState).
  - Manage autorun/auton selection via CyclePrimitives and AutonPreviewer.
  - Run CommandScheduler and per-period updates (RobotState, vision heartbeats, drive-team HUD).
  - Provide extension points for new subsystems and auton sequences.

  Lifecycle summary (where to look)
  - Robot::Robot()          : startup wiring and warm-load resources (e.g., trajectories).
  - Robot::RobotPeriodic()  : runs CommandScheduler, logging (guarded by FMS), RobotState, vision heartbeats, and UpdateDriveTeamFeedback().
  - Robot::AutonomousInit() : raise thread priority, initialize cycle primitives, start auton PeriodicLooper state.
  - Robot::AutonomousPeriodic(): run cycle primitives and auton PeriodicLooper.
  - Robot::TeleopInit()/TeleopPeriodic(): start/run teleop PeriodicLooper state.
  - Robot::TestInit()      : cancel commands at test startup.

  Initialization order (high level)
  1. Logger selections -> dashboard
  2. FieldConstants, RoboRio singletons
  3. ChassisConfigMgr -> CreateDrivetrain()
  4. Instantiate RobotContainer (binds subsystems/commands)
  5. MechanismConfigMgr and RobotState initialization
  6. Create CyclePrimitives and AutonPreviewer for auton selection
  7. Initialize drive-team feedback (DragonField, DriverFeedback)

  Key collaborators / singletons
  - RobotContainer, ChassisConfigMgr, MechanismConfigMgr
  - RobotState, PeriodicLooper
  - CyclePrimitives, AutonPreviewer
  - DriverFeedback, DragonField
  - DragonQuest / DragonVision (vision heartbeats)
  - DragonDataLoggerMgr (optional data logger)

  Extension points
  - Add subsystems and bind them in RobotContainer.
  - Add new CyclePrimitives for auton routines and register them with AutonPreviewer.
  - Use PeriodicLooper for periodic publishing rather than adding heavy work to RobotPeriodic.

  Debugging & testing tips
  - Guard periodic debug logging with !frc::DriverStation::IsFMSAttached() to avoid FMS log flooding.
  - Warm-load trajectories in Robot constructor to avoid first-run stalls.
  - Test thread priority changes on non-competition hardware first.
  - Use PeriodicLooper to throttle heavy publishers and avoid overruns.

  TODO / Caveats
  - Consider re-enabling DragonDataLoggerMgr->PeriodicDataLog() with rate limiting.
  - Move DragonField initialization into a dedicated drive-team feedback manager.
  - Add more granular RobotState transition logging for diagnostics.

  Search tokens in this file
  - InitializeRobot(), InitializeAutonOptions(), InitializeDriveteamFeedback(), UpdateDriveTeamFeedback()
*/

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

#include "RobotContainer.h"
#include "RobotIdentifier.h"
#include "auton/AutonPreviewer.h"
#include "auton/CyclePrimitives.h"
#include "auton/drivePrimitives/AutonUtils.h"
#include "chassis/ChassisConfigMgr.h"
#include "configs/MechanismConfigMgr.h"
#include "feedback/DriverFeedback.h"
#include "fielddata/BargeHelper.h"
#include "fielddata/ReefHelper.h"
#include "frc/DriverStation.h"
#include "frc/RobotController.h"
#include "frc/Threads.h"
#include "state/RobotState.h"
#include "utils/DragonField.h"
#include "utils/PeriodicLooper.h"
#include "utils/RoboRio.h"
#include "utils/logging/debug/Logger.h"
#include "utils/logging/signals/DragonDataLoggerMgr.h"

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
    frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TeleopPeriodic() { PeriodicLooper::GetInstance()->TeleopRunCurrentState(); }

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
