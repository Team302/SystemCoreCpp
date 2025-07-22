$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

#include <string>

#include "configs/MechanismConfig.h"
#include "configs/MechanismConfigMgr.h"
#include "utils/logging/debug/Logger.h"

$$_ROBOT_CONFIG_INCLUDES_$$

using namespace std;

MechanismConfigMgr *MechanismConfigMgr::m_instance = nullptr;
MechanismConfigMgr *MechanismConfigMgr::GetInstance()
{
    if (MechanismConfigMgr::m_instance == nullptr)
    {
        MechanismConfigMgr::m_instance = new MechanismConfigMgr();
    }
    return MechanismConfigMgr::m_instance;
}

MechanismConfigMgr::MechanismConfigMgr() : m_config(nullptr)
{
}

void MechanismConfigMgr::InitRobot(RobotIdentifier id)
{
    switch (id)
    {
        $$_ROBOT_CONFIGURATION_CREATION_$$

    default:
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Skipping robot initialization because of unknown robot id "), string(""), static_cast<int>(id));
        break;
    }

    if (m_config != nullptr)
    {
        m_config->BuildRobot();
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("Initialization completed for robot "), string(""), static_cast<int>(id));
    }
}
