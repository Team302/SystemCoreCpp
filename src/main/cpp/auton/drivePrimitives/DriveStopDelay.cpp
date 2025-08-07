
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

// C++ Includes
#include <string>

// FRC includes
#include "frc/Timer.h"

// Team 302 includes
#include "auton/drivePrimitives/DriveStopDelay.h"
#include "auton/PrimitiveParams.h"
#include "utils/logging/debug/Logger.h"

using namespace std;
using namespace frc;

//========================================================================================================
/// @class  DriveStopDelay
/// @brief  This is an auton primitive that causes the chassis to not drive for a certain amount of time
//========================================================================================================

/// @brief constructor that creates/initializes the object
DriveStopDelay::DriveStopDelay() : DriveStop()
{
}

/// @brief check if the end condition has been met
/// @return bool true means the end condition was reached, false means it hasn't

void DriveStopDelay::Init(PrimitiveParams *params)
{
    DriveStop::Init(params);

    switch (params->GetDelayOption())
    {
    case DelayOption::REEF:
        m_delayTime = params->GetReefDelay();
        break;
    case DelayOption::CORAL_STATION:
        m_delayTime = params->GetCoralStationDelay();
        break;
    default:
        m_delayTime = params->GetStartDelay();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveStopDelay", "Delay", static_cast<double>(m_delayTime));
}

bool DriveStopDelay::IsDone()
{
    return m_timer->AdvanceIfElapsed(m_delayTime) || m_timer->AdvanceIfElapsed(m_maxTime);
}