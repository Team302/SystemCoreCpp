#include "chassis/generated/TunerConstants302.h"
#include "chassis/generated/CommandSwerveDrivetrain.h"

std::unique_ptr<subsystems::CommandSwerveDrivetrain> TunerConstants302::CreateDrivetrain()
{
    return std::make_unique<subsystems::CommandSwerveDrivetrain>(
        TunerConstants302::DrivetrainConstants,
        TunerConstants302::FrontLeft,
        TunerConstants302::FrontRight,
        TunerConstants302::BackLeft,
        TunerConstants302::BackRight);
}
