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

// FRC Includes
//#include "frc/trajectory/TrajectoryUtil.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/trajectory/Trajectory.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/velocity.h"

// Team302 Includes
#include "auton/AutonPreviewer.h"
#include "auton/AutonSelector.h"
#include "auton/PrimitiveEnums.h"
#include "auton/PrimitiveParams.h"
#include "auton/PrimitiveParser.h"
#include "auton/drivePrimitives/AutonUtils.h"
#include "utils/logging/debug/Logger.h"
#include "chassis/ChassisConfigMgr.h"
#include "chassis/generated/CommandSwerveDrivetrain.h"

// Thirdparty includes
using frc::ChassisSpeeds;
using frc::Rotation2d;
using frc::Trajectory;
using std::string;

AutonPreviewer::AutonPreviewer(CyclePrimitives *cyclePrims) : m_selector(cyclePrims->GetAutonSelector()),
                                                              m_prevChoice(""),
                                                              m_field(DragonField::GetInstance())
{
}

void AutonPreviewer::CheckCurrentAuton()
{

    std::string currentChoice = m_selector->GetSelectedAutoFile();

    // If the robot is not disabled, clear the field and return
    // if (!frc::DriverStation::IsDisabled())
    // {
    //     m_field->ResetField();
    //     m_prevChoice = ""; // Optional: force re-population once disabled again
    //     return;
    // }

    // if (currentChoice != m_prevChoice)
    // {
    //     PopulateField();
    //     m_prevChoice = currentChoice;
    // }
}

void AutonPreviewer::PopulateField()
{
    auto trajectories = GetTrajectories();
    m_field->ResetField();
    for (unsigned int i = 0; i < trajectories.size(); i++)
    {
        m_field->AddTrajectory("traj" + std::to_string(i), trajectories[i]);
    }
}

std::vector<frc::Trajectory> AutonPreviewer::GetTrajectories()
{

    std::vector<frc::Trajectory> trajectories;

    ChassisSpeeds speeds;
    speeds.vx = units::velocity::feet_per_second_t(0.0);
    speeds.vy = units::velocity::feet_per_second_t(0.0);
    speeds.omega = units::angular_velocity::degrees_per_second_t(0.0);

    Rotation2d heading(units::angle::degree_t(0.0));

    auto chassis = ChassisConfigMgr::GetInstance()->GetSwerveChassis();

    if (chassis != nullptr)
    {
        auto params = PrimitiveParser::ParseXML(m_selector->GetSelectedAutoFile());

        for (auto param : params)
        {
            std::vector<Trajectory::State> states;

            if (param->GetID() == PRIMITIVE_IDENTIFIER::TRAJECTORY_DRIVE)
            {
                auto pathname = param->GetTrajectoryName();
                /**
                auto path = AutonUtils::GetTrajectoryFromPathFile(pathname);
                if (path.has_value())
                {
                    auto trajectory = path.value();
                    auto endstate = trajectory.GetFinalSample().value();
                    heading = endstate.heading;
                    speeds.vx = endstate.vx;
                    speeds.vy = endstate.vy;

                    auto samples = trajectory.samples;
                    for (auto sample : samples)
                    {
                        Trajectory::State state;
                        state.t = sample.timestamp;
                        state.acceleration = units::math::sqrt(sample.ax * sample.ax + sample.ay * sample.ay);
                        state.velocity = units::math::sqrt(sample.vx * sample.vx + sample.vy * sample.vy);
                        state.pose = sample.GetPose();
                        state.curvature = units::curvature_t(0.1);

                        states.emplace_back(state);
                    }
                    trajectories.emplace_back(states);
                }
                **/
            }
        }
    }
    return trajectories;
}