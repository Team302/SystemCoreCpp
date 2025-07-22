#include "chassis/generated/CommandSwerveDrivetrain.h"
#include <frc/RobotController.h>
#include "utils/FMSData.h"

using namespace subsystems;

void CommandSwerveDrivetrain::Periodic()
{
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || frc::DriverStation::IsDisabled())
    {
        auto const allianceColor = FMSData::GetAllianceColor();

        SetOperatorPerspectiveForward(
            allianceColor == frc::DriverStation::Alliance::kRed
                ? kRedAlliancePerspectiveRotation
                : kBlueAlliancePerspectiveRotation);
        m_hasAppliedOperatorPerspective = true;
    }
}

void CommandSwerveDrivetrain::StartSimThread()
{
    m_lastSimTime = utils::GetCurrentTime();
    m_simNotifier = std::make_unique<frc::Notifier>([this]
                                                    {
        units::second_t const currentTime = utils::GetCurrentTime();
        auto const deltaTime = currentTime - m_lastSimTime;
        m_lastSimTime = currentTime;

        /* use the measured time delta, get battery voltage from WPILib */
        UpdateSimState(deltaTime, frc::RobotController::GetBatteryVoltage()); });
    m_simNotifier->StartPeriodic(kSimLoopPeriod);
}

bool CommandSwerveDrivetrain::IsSamePose()

{
    bool isCurrentlyStopped = GetPose().Translation().Distance(m_prevPose.Translation()) < m_distanceThreshold;

    if (isCurrentlyStopped)
    {
        if (!m_debounceTimer.IsRunning())
        {
            m_debounceTimer.Start();
        }
    }
    else
    {
        m_debounceTimer.Reset();
    }
    m_prevPose = GetPose();

    return m_debounceTimer.HasElapsed(m_samePoseTime);
}