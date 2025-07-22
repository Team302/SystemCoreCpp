
$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

#include <string>

// FRC Includes
#include <networktables/NetworkTableInstance.h>
#include <frc/Timer.h>

#include "$$_MECHANISM_INSTANCE_NAME_$$.h"
#include "utils/logging/debug/Logger.h"
#include "utils/PeriodicLooper.h"
#include "state/RobotState.h"
#include "utils/DragonPower.h"

$$_INCLUDE_FILES_$$

$$_USING_DIRECTIVES_$$

using std::string;
using namespace $$_MECHANISM_INSTANCE_NAME_$$States;

void $$_MECHANISM_INSTANCE_NAME_$$::CreateAndRegisterStates(){
    $$_OBJECT_CREATION_$$

        $$_STATE_TRANSITION_REGISTRATION_$$}

$$_MECHANISM_INSTANCE_NAME_$$::$$_MECHANISM_INSTANCE_NAME_$$(RobotIdentifier activeRobotId) : BaseMech(MechanismTypes::MECHANISM_TYPE::$$_MECHANISM_TYPE_NAME_$$, std::string("$$_MECHANISM_INSTANCE_NAME_$$")),
                                                                                              m_activeRobotId(activeRobotId),
                                                                                              m_stateMap()
{
    PeriodicLooper::GetInstance()->RegisterAll(this);
    InitializeLogging();
}

void $$_MECHANISM_INSTANCE_NAME_$$::InitializeLogging()
{
    wpi::log::DataLog &log = frc::DataLogManager::GetLog();

    $$_DATA_LOGGING_INITIALIZATION_$$
}

std::map<std::string, $$_MECHANISM_INSTANCE_NAME_$$::STATE_NAMES>
    $$_MECHANISM_INSTANCE_NAME_$$::stringToSTATE_NAMESEnumMap{
        $$_STATE_MAP_$$};

$$_CREATE_FUNCTIONS_$$

$$_INITIALZATION_FUNCTIONS_$$

void $$_MECHANISM_INSTANCE_NAME_$$::SetCurrentState(int state, bool run)
{
    StateMgr::SetCurrentState(state, run);
}

void $$_MECHANISM_INSTANCE_NAME_$$::RunCommonTasks()
{
    // This function is called once per loop before the current state Run()
    Cyclic();
}

_STATE_MANAGER_START_
/// @brief  Set the control constants (e.g. PIDF values).
/// @param [in] ControlData*                                   pid:  the control constants
/// @return void
void $$_MECHANISM_INSTANCE_NAME_$$::SetControlConstants(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier, int slot, ControlData pid)
{
}

/// @brief update the output to the mechanism using the current controller and target value(s)
/// @return void
void $$_MECHANISM_INSTANCE_NAME_$$::Update()
{
    $$_CYCLIC_GENERIC_TARGET_REFRESH_$$;
}
_STATE_MANAGER_END_

_MECHANISM_HAS_MOTORS_START_
_STATE_MANAGER_START_
bool $$_MECHANISM_INSTANCE_NAME_$$::IsAtMinPosition(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier) const
{
    // auto motor = GetMotorMech(identifier);
    // if (motor != nullptr)
    // {
    //     return motor->IsAtMinTravel();
    // }
    return false;
}

bool $$_MECHANISM_INSTANCE_NAME_$$::IsAtMaxPosition(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier) const
{
    // auto motor = GetMotorMech(identifier);
    // if (motor != nullptr)
    // {
    //     return motor->IsAtMaxTravel();
    // }
    return false;
}
_STATE_MANAGER_END_

_MECHANISM_HAS_MOTORS_END_

_MECHANISM_HAS_SOLENOIDS_START_
_STATE_MANAGER_START_
void $$_MECHANISM_INSTANCE_NAME_$$::UpdateTarget(RobotElementNames::SOLENOID_USAGE identifier, bool extend)
{
    auto sol = GetSolenoidMech(identifier);
    if (sol != nullptr)
    {
        sol->ActivateSolenoid(extend);
    }
}

bool $$_MECHANISM_INSTANCE_NAME_$$::IsAtMinPosition(RobotElementNames::SOLENOID_USAGE identifier) const
{
    // auto sol = GetSolenoidMech(identifier);
    // if (sol != nullptr)
    // {
    //     return !sol->IsSolenoidActivated();
    // }
    return false;
}

bool $$_MECHANISM_INSTANCE_NAME_$$::IsAtMaxPosition(RobotElementNames::SOLENOID_USAGE identifier) const
{
    // auto sol = GetSolenoidMech(identifier);
    // if (sol != nullptr)
    // {
    //     return sol->IsSolenoidActivated();
    // }
    return false;
}
_STATE_MANAGER_END_

_MECHANISM_HAS_SOLENOIDS_END_

void $$_MECHANISM_INSTANCE_NAME_$$::Cyclic()
{
    Update();

    _NT_TUNING_FUNCTION_CALLS_START_
    CheckForTuningEnabled();
    if (m_tuning)
    {
        ReadTuningParamsFromNT();
    }
    _NT_TUNING_FUNCTION_CALLS_END_
}

_NT_TUNING_FUNCTIONS_START_
void $$_MECHANISM_INSTANCE_NAME_$$::CheckForTuningEnabled()
{
    bool pastTuning = m_tuning;
    m_tuning = m_table.get()->GetBoolean(m_tuningIsEnabledStr, false);
    if (pastTuning != m_tuning && m_tuning == true)
    {
        PushTuningParamsToNT();
    }
}

void $$_MECHANISM_INSTANCE_NAME_$$::ReadTuningParamsFromNT()
{
    $$_READ_TUNABLE_PARAMETERS_$$
}

void $$_MECHANISM_INSTANCE_NAME_$$::PushTuningParamsToNT(){
    $$_PUSH_TUNABLE_PARAMETERS_$$}

_NT_TUNING_FUNCTIONS_END_

    ControlData *$$_MECHANISM_INSTANCE_NAME_$$::GetControlData(string name)
{
    $$_CONTROLDATA_NAME_TO_VARIABLE_$$

    return nullptr;
}

void $$_MECHANISM_INSTANCE_NAME_$$::DataLog(uint64_t timestamp)
{
    $$_DATALOG_METHOD_$$
}