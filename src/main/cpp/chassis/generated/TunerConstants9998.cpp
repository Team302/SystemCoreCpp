#include "chassis/generated/TunerConstants9998.h"
#include "chassis/generated/CommandSwerveDrivetrain.h"

std::unique_ptr<subsystems::CommandSwerveDrivetrain> TunerConstants9998::CreateDrivetrain()
{
    return std::make_unique<subsystems::CommandSwerveDrivetrain>(
        TunerConstants9998::DrivetrainConstants,
        TunerConstants9998::FrontLeft,
        TunerConstants9998::FrontRight,
        TunerConstants9998::BackLeft,
        TunerConstants9998::BackRight);
}
