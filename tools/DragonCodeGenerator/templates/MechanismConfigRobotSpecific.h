$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

#pragma once
#include "configs/MechanismConfig.h"
#include "mechanisms/MechanismTypes.h"
#include "state/StateMgr.h"
$$_MECHANISM_INCLUDE_FILES_$$

class MechanismConfig$$_ROBOT_NAME_$$ : public MechanismConfig
{
public:
    MechanismConfig$$_ROBOT_NAME_$$() = default;
    ~MechanismConfig$$_ROBOT_NAME_$$() = default;

    StateMgr *GetMechanism(MechanismTypes::MECHANISM_TYPE mechType);

protected:
    void DefineMechanisms() override;
    void DefineLEDs() override;

private:
    $$_MECHANISM_PTR_DECLARATIONS_$$

    std::unordered_map<MechanismTypes::MECHANISM_TYPE, StateMgr *> m_mechanismMap;
};