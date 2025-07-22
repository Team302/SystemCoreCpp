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

#include <string>

#include "utils/logging/debug/Logger.h"
#include "vision/definitions/CameraConfig_302.h"
#include "vision/definitions/CameraConfig_9997.h"
#include "vision/definitions/CameraConfig_9999.h"
#include "vision/definitions/CameraConfig.h"
#include "vision/definitions/CameraConfigMgr.h"

using namespace std;

CameraConfigMgr *CameraConfigMgr::m_instance = nullptr;
CameraConfigMgr *CameraConfigMgr::GetInstance()
{
    if (CameraConfigMgr::m_instance == nullptr)
    {
        CameraConfigMgr::m_instance = new CameraConfigMgr();
    }
    return CameraConfigMgr::m_instance;
}

CameraConfigMgr::CameraConfigMgr() : m_config(nullptr)
{
}

void CameraConfigMgr::InitCameras(RobotIdentifier id)
{
    switch (id)
    {

    case RobotIdentifier::COMP_BOT_302:
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("Camera Init"), string("Success"), static_cast<int>(id));
        m_config = new CameraConfig_302();
        break;

    case RobotIdentifier::CHASSIS_BOT_9997:
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("Camera Init"), string("Success"), static_cast<int>(id));
        m_config = new CameraConfig_9997();
        break;

    case RobotIdentifier::CHASSIS_BOT_9998:
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("Camera Init"), string("Success"), static_cast<int>(id));
        m_config = new CameraConfig_9997();
        break;

    case RobotIdentifier::PRACTICE_BOT_9999:
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("Camera Init"), string("Success"), static_cast<int>(id));
        m_config = new CameraConfig_9999();
        break;

    case RobotIdentifier::SIM_BOT_0:
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("Camera Init"), string("Success"), static_cast<int>(id));
        m_config = new CameraConfig_302();
        break;

    default:
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("Camera Init"), string("Skipping camera initialization because of unknown robot id "), static_cast<int>(id));
        break;
    }

    if (m_config != nullptr)
    {
        m_config->BuildCameraConfig();
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("Camera Init"), string("Initialization completed for robot cameras "), static_cast<int>(id));
    }
}
