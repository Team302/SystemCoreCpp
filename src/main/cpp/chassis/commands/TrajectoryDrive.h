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

#include "frc2/command/CommandHelper.h"
#include "frc2/command/Command.h"
#include "chassis/generated/CommandSwerveDrivetrain.h"

// FRC Includes
#include "frc/Timer.h"
#include "frc/controller/PIDController.h"
#include <choreo/trajectory/Trajectory.h>

class TrajectoryDrive : public frc2::CommandHelper<frc2::Command, TrajectoryDrive>
{
public:
    /**
     * @brief Construct a new Trajectory Drive command
     *
     * @param chassis The swerve drive subsystem
     * @param trajectory The Choreo trajectory to follow
     */
    explicit TrajectoryDrive(subsystems::CommandSwerveDrivetrain *chassis);

    // FRC Command Lifecycle methods
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

    void SetPath(const std::string &pathName);

    std::string WhyDone() const { return m_whyDone; };

    units::time::second_t GetTotalTrajectoryTime() const { return m_totalTrajectoryTime; }

private:
    subsystems::CommandSwerveDrivetrain *m_chassis;
    std::string m_pathName;

    bool IsSamePose(frc::Pose2d currentPose, frc::Pose2d previousPose, frc::ChassisSpeeds velocity, double xyTolerance, double rotTolerance, double speedTolerance);

    std::optional<choreo::Trajectory<choreo::SwerveSample>> m_trajectory;
    choreo::SwerveSample m_finalState;
    std::vector<choreo::SwerveSample> m_trajectoryStates;

    frc::Pose2d m_prevPose;
    bool m_wasMoving;
    frc::Transform2d m_delta;
    std::unique_ptr<frc::Timer> m_timer;

    std::string m_whyDone;
    units::time::second_t m_totalTrajectoryTime;

    static constexpr double kPDrive{0.65};
    static constexpr double kIDrive{0.0};
    static constexpr double kDDrive{0.0};

    static constexpr double kPHeading{1.0};
    static constexpr double kIHeading{0.0};
    static constexpr double kDHeading{0.0};

    frc::PIDController m_xController{kPDrive, kIDrive, kDDrive};
    frc::PIDController m_yController{kPDrive, kIDrive, kDDrive};
    frc::PIDController m_headingController{kPHeading, kIHeading, kDHeading};

    frc::ChassisSpeeds m_chassisSpeeds;

    swerve::requests::FieldCentric m_driveRequest;
};
