
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

// Team 302 includes
#include "auton/drivePrimitives/IPrimitive.h"
#include "auton/drivePrimitives/ResetPositionTrajectory.h"
#include "auton/drivePrimitives/VisionDrivePrimitive.h"
#include "auton/PrimitiveEnums.h"
#include "auton/PrimitiveFactory.h"
#include "auton/drivePrimitives/AutonDrivePrimitive.h"

PrimitiveFactory *PrimitiveFactory::m_instance = nullptr;

PrimitiveFactory *PrimitiveFactory::GetInstance()
{
    if (PrimitiveFactory::m_instance == nullptr)
    {                                                          // If we do not have an instance
        PrimitiveFactory::m_instance = new PrimitiveFactory(); // Create a new instance
    }
    return PrimitiveFactory::m_instance; // Return said instance
}

PrimitiveFactory::PrimitiveFactory() : m_resetPositionTrajectory(nullptr),
                                       m_autonDrivePrimitive(nullptr)
{
}

PrimitiveFactory::~PrimitiveFactory()
{
    PrimitiveFactory::m_instance = nullptr;
}

IPrimitive *PrimitiveFactory::GetIPrimitive(PrimitiveParams *primitivePasser)
{
    IPrimitive *primitive = nullptr;
    switch (primitivePasser->GetID()) // Decides which primitive to get or make
    {
    case DO_NOTHING:
    case DO_NOTHING_MECHANISMS:
    case HOLD_POSITION:
    case TRAJECTORY_DRIVE:
    case VISION_ALIGN:
        if (m_autonDrivePrimitive == nullptr)
        {
            m_autonDrivePrimitive = new AutonDrivePrimitive();
        }
        primitive = m_autonDrivePrimitive;
        break;

    case RESET_POSITION:
        if (m_resetPositionTrajectory == nullptr)
        {
            m_resetPositionTrajectory = new ResetPositionTrajectory();
        }
        primitive = m_resetPositionTrajectory;
        break;

    default:
        break;
    }

    return primitive;
}
