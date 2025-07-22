$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

// C++ Includes
#include <string>

// FRC includes

// Team 302 includes
#include "mechanisms/$$_MECHANISM_INSTANCE_NAME_$$/$$_STATE_NAME_$$State.h"
#include "teleopcontrol/TeleopControl.h"
#include "teleopcontrol/TeleopControlFunctions.h"
#include "utils/logging/debug/Logger.h"

// Third Party Includes

using namespace std;
using namespace $$_MECHANISM_INSTANCE_NAME_$$States;

/// @class ExampleForwardState
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
$$_STATE_NAME_$$State::$$_STATE_NAME_$$State(std::string stateName,
                                             int stateId,
                                             $$_MECHANISM_INSTANCE_NAME_$$ *mech,
                                             RobotIdentifier activeRobotId) : State(stateName, stateId), m_mechanism(mech), m_RobotId(activeRobotId)
{
}

void $$_STATE_NAME_$$State::Init()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("$$_STATE_NAME_$$State"), string("Init"));

    $$_STATE_INIT_FUNCTION_CALLS_$$
}

$$_STATE_INIT_FUNCTIONS_$$

void $$_STATE_NAME_$$State::Run()
{
    // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("$$_STATE_NAME_$$State"), string("Run"));
}

void $$_STATE_NAME_$$State::Exit()
{
    // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("$$_STATE_NAME_$$State"), string("Exit"));
}

bool $$_STATE_NAME_$$State::AtTarget()
{
    // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("$$_STATE_NAME_$$State"), string("AtTarget"));

    bool atTarget = false;
    return atTarget;
}

bool $$_STATE_NAME_$$State::IsTransitionCondition(bool considerGamepadTransitions)
{
    // To get the current state use m_mechanism->GetCurrentState()
    return false;
    // return (considerGamepadTransitions && TeleopControl::GetInstance()->IsButtonPressed(TeleopControlFunctions::EXAMPLE_MECH_FORWARD));
}
