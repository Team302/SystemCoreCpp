$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

#include <string>

#include "configs/MechanismConfig$$_ROBOT_NAME_$$.h"
#include "configs/MechanismConfigMgr.h"
#include "configs/RobotElementNames.h"
#include "utils/logging/debug/Logger.h"
#include "utils/PeriodicLooper.h"

$$_INCLUDE_$$

using std::string;

void MechanismConfig$$_ROBOT_NAME_$$::DefineMechanisms(){
    $$_MECHANISMS_INITIALIZATION_$$}

StateMgr *MechanismConfig$$_ROBOT_NAME_$$::GetMechanism(MechanismTypes::MECHANISM_TYPE mechType)
{
    auto itr = m_mechanismMap.find(mechType);
    if (itr != m_mechanismMap.end())
    {
        return itr->second;
    }
    return nullptr;
}

void MechanismConfig$$_ROBOT_NAME_$$::DefineLEDs()
{
    $$_LED_INITIALIZATION_$$
}