
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

#pragma once


// FRC includes
#include "units/time.h"

// Team 302 includes
#include "auton/drivePrimitives/DriveStop.h"

// Third Party Includes

//========================================================================================================
/// @class  DriveStop
/// @brief  This is an auton primitive that causes the chassis to not drive
//========================================================================================================

class DriveStopDelay : public DriveStop
{
public:
    enum DelayOption
    {
        START,
        REEF,
        CORAL_STATION,
        MAX_OPTIONS
    };
    /// @brief constructor that creates/initializes the object
    DriveStopDelay();

    /// @brief destructor, clean  up the memory from this object
    virtual ~DriveStopDelay() = default;

    /// @brief check if the end condition has been met
    /// @return bool true means the end condition was reached, false means it hasn't
    bool IsDone() override;
    void Init(PrimitiveParams *params) override;

private:
    units::time::second_t m_delayTime;
};
