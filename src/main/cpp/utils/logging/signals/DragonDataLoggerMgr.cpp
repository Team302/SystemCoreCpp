
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

#include <filesystem>

#include "ctre/phoenix6/SignalLogger.hpp"
#include "frc/DataLogManager.h"
#include "frc/DriverStation.h"
#include "frc/RobotController.h"
#include "utils/logging/signals/DragonDataLoggerMgr.h"
#include "utils/logging/signals/DragonDataLoggerSignals.h"

using namespace std;
using ctre::phoenix6::SignalLogger;

DragonDataLoggerMgr *DragonDataLoggerMgr::m_instance = nullptr;
DragonDataLoggerMgr *DragonDataLoggerMgr::GetInstance()
{
    if (DragonDataLoggerMgr::m_instance == nullptr)
    {
        DragonDataLoggerMgr::m_instance = new DragonDataLoggerMgr();
    }
    return DragonDataLoggerMgr::m_instance;
}

DragonDataLoggerMgr::DragonDataLoggerMgr() : m_items() //, m_doubleDatalogSignals(), m_boolDatalogSignals(), m_stringDatalogSignals()
{
    // auto logFolder = GetLoggingDir();
    // // frc::DataLogManager::Start(logFolder, CreateLogFileName());
    // frc::DataLogManager::Start();
    // frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
    // DragonDataLoggerSignals::GetInstance();

    // SignalLogger::SetPath(logFolder.c_str());
    // SignalLogger::EnableAutoLogging(true);
    // SignalLogger::Start();

    SignalLogger::SetPath(GetLoggingDir().c_str());
    SignalLogger::EnableAutoLogging(true);
    SignalLogger::Start();
    m_timer.Start();
}

DragonDataLoggerMgr::~DragonDataLoggerMgr()
{
    SignalLogger::Stop();
}

/**
 * @brief Create a log file name based on the current date and time
 */
std::string DragonDataLoggerMgr::CreateLogFileName()
{
    time_t now = time(0);
    tm *ltm = localtime(&now);
    char buffer[80];
    strftime(buffer, 80, "%Y%m%d-%H%M%S", ltm);
    string time(buffer);

    string filename = "frc302-" + time + ".wpilog";
    return filename;
}

std::string DragonDataLoggerMgr::GetLoggingDir()
{
    // check if usb log directory exists
    if (std::filesystem::exists("/media/sda1/logs/"))
    {
        return std::filesystem::path("/media/sda1/logs/").string();
    }
    else if (std::filesystem::exists("/home/lvuser/logs/"))
    {
        return std::filesystem::path("/home/lvuser/logs/").string();
    }
    else if (std::filesystem::exists("/home/systemcore/logs/"))
    {
        return std::filesystem::path("/home/systemcore/logs/").string();
    }

    return std::string("");
}

void DragonDataLoggerMgr::RegisterItem(DragonDataLogger *item)
{
    m_items.emplace_back(item);
}

void DragonDataLoggerMgr::PeriodicDataLog()
{
    uint64_t timestamp = frc::RobotController::GetFPGATime();

    m_timer.Reset();
    while (m_timer.Get() < m_period)
    {
        auto item = m_items[m_lastIndex];
        item->DataLog(timestamp);
        m_lastIndex += ((m_lastIndex >= (m_items.size() - 1)) ? -m_lastIndex : 1);
    }
    // for (auto item : m_items)
    //{
    //     item->DataLog(timestamp);
    // }
    //  wpi::log::DataLog &log = frc::DataLogManager::GetLog();
    //  log.Flush();
}
