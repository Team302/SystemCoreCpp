
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

// C++ Includes
#include <string>
#include <vector>

// FRC includes
#include "units/time.h"

// Team 302 includes
#include "auton/PrimitiveEnums.h"
#include "chassis/ChassisOptionEnums.h"
#include "vision/DragonVision.h"
#include "auton/ZoneParams.h"
#include "auton/drivePrimitives/DriveStopDelay.h"
#include "mechanisms/DragonTale/DragonTale.h"

// Third Party Includes

class PrimitiveParams
{
public:
    enum VISION_ALIGNMENT
    {
        UNKNOWN = -1,
        ALGAE = DragonVision::VISION_ELEMENT::ALGAE,
        CORAL_STATION = DragonVision::VISION_ELEMENT::CORAL_STATION,
        REEF = DragonVision::VISION_ELEMENT::REEF,
        PROCESSOR = DragonVision::VISION_ELEMENT::PROCESSOR
    };
    // @ADDMECH add parameter for your mechanism state
    PrimitiveParams(PRIMITIVE_IDENTIFIER id,
                    units::time::second_t time,
                    ChassisOptionEnums::HeadingOption headingOption,
                    float heading,
                    std::string pathName,
                    std::string choreoTrajectoryName,
                    ChassisOptionEnums::PathGainsType pahtgainsType,
                    ZoneParamsVector zones, // create zones parameter of type
                    VISION_ALIGNMENT visionAlignment,
                    bool changeTaleState,
                    DragonTale::STATE_NAMES taleState,
                    ChassisOptionEnums::DriveStateType pathUpdateOption,
                    DriveStopDelay::DelayOption delayOption); // create zones parameter of type ZonesParamsVector

    PrimitiveParams() = delete;
    virtual ~PrimitiveParams() = default; // Destructor

    // Some getters
    PRIMITIVE_IDENTIFIER GetID() const { return m_id; };
    ChassisOptionEnums::DriveStateType GetPathUpdateOption() const { return m_pathUpdateOption; }

    units::time::second_t GetTime() const { return m_time; };
    ChassisOptionEnums::HeadingOption GetHeadingOption() const { return m_headingOption; };
    float GetHeading() const { return m_heading; };
    std::string GetPathName() const { return m_pathName; };
    std::string GetTrajectoryName() const { return m_choreoTrajectoryName; };
    ChassisOptionEnums::PathGainsType GetPathGainsType() const { return m_pathGainsType; }
    ZoneParamsVector GetZones() const { return m_zones; }; // create a GetZones() method to return the instance of zones m_zones
    VISION_ALIGNMENT GetVisionAlignment() const { return m_visionAlignment; }

    DriveStopDelay::DelayOption GetDelayOption() const { return m_delayOption; }

    void SetStartDelay(units::time::second_t startDelay) { m_startDelay = startDelay; }
    void SetReefDelay(units::time::second_t reefDelay) { m_reefDelay = reefDelay; }
    void SetCoralStationDelay(units::time::second_t coralStationDelay) { m_coralStationDelay = coralStationDelay; }
    units::time::second_t GetStartDelay() const { return m_startDelay; }
    units::time::second_t GetReefDelay() const { return m_reefDelay; }
    units::time::second_t GetCoralStationDelay() const { return m_coralStationDelay; }

    bool IsTaleStateChanging() const { return m_changeTaleState; }
    DragonTale::STATE_NAMES GetTaleState() const { return m_taleState; }
    // Setters
    void SetPathName(std::string path)
    {
        m_pathName = path;
    }
    void SetVisionAlignment(VISION_ALIGNMENT visionAlignment) { m_visionAlignment = visionAlignment; }

private:
    // Primitive Parameters
    PRIMITIVE_IDENTIFIER m_id; // Primitive ID
    units::time::second_t m_time;
    ChassisOptionEnums::HeadingOption m_headingOption = ChassisOptionEnums::HeadingOption::IGNORE;
    float m_heading;

    DriveStopDelay::DelayOption m_delayOption;

    units::time::second_t m_startDelay;
    units::time::second_t m_reefDelay;
    units::time::second_t m_coralStationDelay;

    std::string m_pathName;
    std::string m_choreoTrajectoryName;
    ChassisOptionEnums::PathGainsType m_pathGainsType;
    VISION_ALIGNMENT m_visionAlignment;

    bool m_changeTaleState;

    DragonTale::STATE_NAMES m_taleState;

    ZoneParamsVector m_zones;

    ChassisOptionEnums::DriveStateType m_pathUpdateOption;
};

typedef std::vector<PrimitiveParams *> PrimitiveParamsVector;
