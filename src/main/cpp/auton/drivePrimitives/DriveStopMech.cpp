
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

// FRC includesssss

// Team 302 includes
#include "auton/drivePrimitives/DriveStop.h"
#include "auton/drivePrimitives/DriveStopMech.h"
#include "auton/PrimitiveParams.h"
#include "configs/MechanismConfig.h"
#include "configs/MechanismConfigMgr.h"

// #include "utils/logging/debug/Logger.h"

// Third Party Includes

using namespace std;
using namespace frc;

//========================================================================================================
/// @class  DriveStop
/// @brief  This is an auton primitive that causes the chassis to not drive
//========================================================================================================

/// @brief constructor that creates/initializes the object
DriveStopMech::DriveStopMech() : DriveStop()
{
    auto config = MechanismConfigMgr::GetInstance()->GetCurrentConfig();
    auto dragonTale = config != nullptr ? config->GetMechanism(MechanismTypes::DRAGON_TALE) : nullptr;

    m_dragonTaleMgr = dragonTale != nullptr ? dynamic_cast<DragonTale *>(dragonTale) : nullptr;
}
void DriveStopMech::Init(PrimitiveParams *params)
{
    DriveStop::Init(params);
    auto config = MechanismConfigMgr::GetInstance()->GetCurrentConfig();
    auto dragonTale = config != nullptr ? config->GetMechanism(MechanismTypes::DRAGON_TALE) : nullptr;
    m_dragonTaleMgr = dragonTale != nullptr ? dynamic_cast<DragonTale *>(dragonTale) : nullptr;
}

/// @brief check if the end condition has been met
/// @return bool true means the end condition was reached, false means it hasn't
bool DriveStopMech::IsDone()
{
    if (m_dragonTaleMgr != nullptr)
    {
        return m_dragonTaleMgr->GetCurrentState() == DragonTale::STATE_NAMES::STATE_READY ||
               m_dragonTaleMgr->GetCurrentState() == DragonTale::STATE_NAMES::STATE_HOLD ||
               DriveStop::IsDone();
    }
    return DriveStop::IsDone();
}
