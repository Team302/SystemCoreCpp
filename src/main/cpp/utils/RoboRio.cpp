
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

#include "frc/RobotController.h"
#include "units/voltage.h"
#include "utils/RoboRio.h"

RoboRio *RoboRio::m_instance = nullptr;
RoboRio *RoboRio::GetInstance()
{
    if (RoboRio::m_instance == nullptr)
    {
        RoboRio::m_instance = new RoboRio();
    }
    return RoboRio::m_instance;
}

void RoboRio::DataLog(uint64_t timestamp)
{
    LogDoubleData(timestamp, DragonDataLogger::DoubleSignals::BATTERY_VOLTAGE, frc::RobotController::GetBatteryVoltage().value());
    LogDoubleData(timestamp, DragonDataLogger::DoubleSignals::INPUT_VOLTAGE, frc::RobotController::GetInputVoltage());
    //LogDoubleData(timestamp, DragonDataLogger::DoubleSignals::INPUT_CURRENT, frc::RobotController::GetInputCurrent());
    LogDoubleData(timestamp, DragonDataLogger::DoubleSignals::CPU_TEMP, frc::RobotController::GetCPUTemp().value());

    LogBoolData(timestamp, DragonDataLogger::BoolSignals::IS_BROWNOUT, frc::RobotController::IsBrownedOut());

    /** other things we may want to add, but commenting out for now
    auto commsDisabledCount = frc::RobotController::GetCommsDisableCount();
    auto isRSLOn = frc::RobotController::GetRSLState();


    auto radioLEDColor = frc::RobotController::GetRadioLEDState();
    auto radioLEDStatus = radioLEDColor == frc::RadioLEDState::kOff ? std::string("Off") : radioLEDColor == frc::RadioLEDState::kGreen ? std::string("ON")
                                                                                       : radioLEDColor == frc::RadioLEDState::kRed     ? std::string("RED")
                                                                                                                                       : std::string("ORANGE");

    auto canStatus = frc::RobotController::GetCANStatus();
    auto rioCANBusUtil = canStatus::percentBusUtilization;
    auto rioCANBusOffCount = canStatus::busOffCount;
    auto rioCANBusTxFullCount = canStatus::txFullCount;
    auto rioCANReceiveErrors = canStatus::receiveErrorCount;
    auto rioCANTransmitErrors = canStatus::transmitErrorCount;

    auto isRail33VEnabled = frc::RobotController::GetEnabled3V3();
    auto rail33VVoltage = frc::RobotController::GetVoltage3V3();
    auto rail33VCurrent = frc::RobotController::GetCurrent3V3();
    auto rail33VFaultCount = frc::RobotController::GetFaultCount3V3();

    auto isRail5VEnabled = frc::RobotController::GetEnabled5V();
    auto rail5VVoltage = frc::RobotController::GetVoltage5V();
    auto rail5VCurrent = frc::RobotController::GetCurrent5V();
    auto rail5VFaultCount = frc::RobotController::GetFaultCount5V();

    auto isRail6VEnabled = frc::RobotController::GetEnabled6V();
    auto rail6VVoltage = frc::RobotController::GetVoltaged6V();
    auto rail6VCurrent = frc::RobotController::GetCurrent6V();
    auto rail6VFaultCount = frc::RobotController::GetFaultCount6V();
    **/
}
