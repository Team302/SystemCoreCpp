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

#include <memory>

#include "frc2/command/CommandPtr.h"
#include "frc2/command/button/CommandXboxController.h"
#include "chassis/generated/CommandSwerveDrivetrain.h"
#include "state/IRobotStateChangeSubscriber.h"
#include "chassis/generated/Telemetry.h"
#include "chassis/ChassisConfigMgr.h"
#include "teleopcontrol/TeleopControl.h"
#include "chassis/commands/TrajectoryDrive.h"

class SwerveContainer : IRobotStateChangeSubscriber
{
public:
    static SwerveContainer *GetInstance();

    frc2::Command *GetDriveToCoralStationCommand() { return m_driveToCoralStationSidewall.get(); }
    frc2::Command *GetDriveToCoralLeftBranchCommand() { return m_driveToCoralLeftBranch.get(); }
    frc2::Command *GetDriveToCoralRightBranchCommand() { return m_driveToCoralRightBranch.get(); }
    frc2::Command *GetDriveToBargeCommand() { return m_driveToBarge.get(); }
    TrajectoryDrive *GetTrajectoryDriveCommand() { return m_trajectoryDrive.get(); }

private:
    SwerveContainer();
    virtual ~SwerveContainer() = default;
    static SwerveContainer *m_instance;

    subsystems::CommandSwerveDrivetrain *m_chassis;

    units::meters_per_second_t m_maxSpeed;
    static constexpr units::radians_per_second_t m_maxAngularRate{1.5_tps};

    Telemetry logger;

    frc2::CommandPtr m_fieldDrive;
    frc2::CommandPtr m_robotDrive;
    frc2::CommandPtr m_driveToCoralStationSidewall;
    frc2::CommandPtr m_driveToCoralStationAlliance;
    frc2::CommandPtr m_driveToCoralRightBranch;
    frc2::CommandPtr m_driveToCoralLeftBranch;
    frc2::CommandPtr m_driveToBarge;
    frc2::CommandPtr m_driveToLeftCage;
    frc2::CommandPtr m_driveToRightCage;
    frc2::CommandPtr m_driveToCenterCage;
    frc2::CommandPtr m_driveToAlgae;
    std::unique_ptr<TrajectoryDrive> m_trajectoryDrive;

    bool m_climbMode = false;
    RobotStateChanges::DesiredCoralSide m_desiredCoralSide = RobotStateChanges::DesiredCoralSide::Sidewall;

    void ConfigureBindings();
    void SetSysIDBinding(TeleopControl *controller);
    void CreateStandardDriveCommands(TeleopControl *controller);
    void CreateReefscapeDriveToCommands(TeleopControl *controller);

    void NotifyStateUpdate(RobotStateChanges::StateChange change, int value) override;
};
