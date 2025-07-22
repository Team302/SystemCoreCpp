$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

#pragma once

#include "configs/MechanismConfig.h"
#include "RobotIdentifier.h"
class MechanismConfigMgr
{
public:
    static MechanismConfigMgr *GetInstance();
    MechanismConfig *GetCurrentConfig() const { return m_config; }
    void InitRobot(RobotIdentifier);

private:
    MechanismConfigMgr();
    ~MechanismConfigMgr() = default;

    static MechanismConfigMgr *m_instance;
    MechanismConfig *m_config;
};
