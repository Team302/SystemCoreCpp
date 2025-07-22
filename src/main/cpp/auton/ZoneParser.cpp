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

#include <map>
#include <string>

#include "auton/AutonGrid.h"
#include "auton/PrimitiveEnums.h"
#include "auton/ZoneParams.h"
#include "auton/ZoneParser.h"
#include "frc/Filesystem.h"
#include "mechanisms/DragonTale/DragonTale.h"
#include "pugixml/pugixml.hpp"
#include "utils/logging/debug/Logger.h"

using namespace std;
using namespace pugi;

ZoneParams *ZoneParser::ParseXML(string fulldirfile)
{
    auto hasError = false;

    static robin_hood::unordered_map<std::string, ChassisOptionEnums::AutonChassisOptions> xmlStringToChassisOptionEnumMap{
        {"VISION_DRIVE_SPEAKER", ChassisOptionEnums::AutonChassisOptions::VISION_DRIVE_SPEAKER},
        {"NO_VISION", ChassisOptionEnums::AutonChassisOptions::NO_VISION},
    };

    static robin_hood::unordered_map<std::string, ChassisOptionEnums::HeadingOption> xmlStringToHeadingOptionEnumMap{

        {"MAINTAIN", ChassisOptionEnums::HeadingOption::MAINTAIN},
        {"SPECIFIED_ANGLE", ChassisOptionEnums::HeadingOption::SPECIFIED_ANGLE},
        {"FACE_GAME_PIECE", ChassisOptionEnums::HeadingOption::FACE_GAME_PIECE},
        {"FACE_REEF_CENTER", ChassisOptionEnums::HeadingOption::FACE_REEF_CENTER},
        {"FACE_REEF_FACE", ChassisOptionEnums::HeadingOption::FACE_REEF_FACE},
        {"FACE_CORAL_STATION", ChassisOptionEnums::HeadingOption::FACE_CORAL_STATION},
        {"IGNORE", ChassisOptionEnums::HeadingOption::IGNORE}};

    static robin_hood::unordered_map<string, ChassisOptionEnums::DriveStateType> xmlStringToPathUpdateOptionMap{{"RIGHT_REEF_BRANCH", ChassisOptionEnums::DRIVE_TO_RIGHT_REEF_BRANCH},
                                                                                                                {"LEFT_REEF_BRANCH", ChassisOptionEnums::DRIVE_TO_LEFT_REEF_BRANCH},
                                                                                                                //    {"REEF_ALGAE", PATH_UPDATE_OPTION::REEF_ALGAE},
                                                                                                                //    {"FLOOR_ALGAE", PATH_UPDATE_OPTION::FLOOR_ALGAE},
                                                                                                                {"CORAL_STATION", ChassisOptionEnums::DRIVE_TO_CORAL_STATION},
                                                                                                                //    {"PROCESSOR", PATH_UPDATE_OPTION::PROCESSOR},
                                                                                                                {"BARGE", ChassisOptionEnums::DRIVE_TO_BARGE},
                                                                                                                {"NOTHING", ChassisOptionEnums::STOP_DRIVE}};

    static robin_hood::unordered_map<std::string, ChassisOptionEnums::AutonAvoidOptions> xmlStringToAvoidOptionEnumMap{
        {"PODIUM", ChassisOptionEnums::AutonAvoidOptions::PODIUM},
        {"ROBOT_COLLISION", ChassisOptionEnums::AutonAvoidOptions::ROBOT_COLLISION},
        {"NO_AVOID_OPTION", ChassisOptionEnums::AutonAvoidOptions::NO_AVOID_OPTION},

    };

    auto deployDir = frc::filesystem::GetDeployDirectory();
    auto zonedir = deployDir + "/auton/zones/";

    string updfulldirfile = zonedir;
    updfulldirfile += fulldirfile;

    xml_document doc;
    xml_parse_result result = doc.load_file(updfulldirfile.c_str());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PrimitiveParser", "updated File", updfulldirfile.c_str());
    if (result)
    {
        xml_node auton = doc.root();
        for (xml_node zonenode = auton.first_child().first_child(); zonenode; zonenode = zonenode.next_sibling())
        {

            double radius = -1;
            double circleX = -1;
            double circleY = -1;

            units::length::meter_t xgrid1rect = units::length::meter_t(0.0);
            units::length::meter_t ygrid1rect = units::length::meter_t(0.0);
            units::length::meter_t xgrid2rect = units::length::meter_t(0.0);
            units::length::meter_t ygrid2rect = units::length::meter_t(0.0);

            ZoneMode zoneMode = ZoneMode::NOTHING;

            // TODO: add zoneType parsing and check

            ChassisOptionEnums::AutonChassisOptions chassisChosenOption = ChassisOptionEnums::AutonChassisOptions::NO_VISION;
            bool isTaleStateChanging = false;
            DragonTale::STATE_NAMES taleChosenOption = DragonTale::STATE_NAMES::STATE_READY;
            ChassisOptionEnums::HeadingOption chosenHeadingOption = ChassisOptionEnums::HeadingOption::IGNORE;

            ChassisOptionEnums::DriveStateType chosenUpdateOption = ChassisOptionEnums::STOP_DRIVE;
            ChassisOptionEnums::AutonAvoidOptions avoidChosenOption = ChassisOptionEnums::AutonAvoidOptions::NO_AVOID_OPTION;

            auto config = MechanismConfigMgr::GetInstance()->GetCurrentConfig();

            // looping through the zone xml attributes to define the location of a given zone (based on 2 sets grid coordinates)
            for (xml_attribute attr = zonenode.first_attribute(); attr; attr = attr.next_attribute())
            {

                if (strcmp(attr.name(), "circlex") == 0)
                {
                    zoneMode = ZoneMode::CIRCLE;
                    circleX = attr.as_double();
                    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ZoneParser", "parsed circlex", circleX);
                }
                else if (strcmp(attr.name(), "circley") == 0)
                {

                    zoneMode = ZoneMode::CIRCLE;
                    circleY = attr.as_double();
                    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ZoneParser", "parsed circley", circleX);
                }
                else if (strcmp(attr.name(), "radius") == 0)
                {
                    zoneMode = ZoneMode::CIRCLE;
                    radius = attr.as_double();
                }
                if (strcmp(attr.name(), "x1_rect") == 0)
                {
                    zoneMode = ZoneMode::RECTANGLE;
                    xgrid1rect = units::length::meter_t(attr.as_double());
                }
                else if (strcmp(attr.name(), "y1_rect") == 0)
                {
                    zoneMode = ZoneMode::RECTANGLE;
                    ygrid1rect = units::length::meter_t(attr.as_double());
                }
                else if (strcmp(attr.name(), "x2_rect") == 0)
                {
                    zoneMode = ZoneMode::RECTANGLE;
                    xgrid2rect = units::length::meter_t(attr.as_double());
                }
                else if (strcmp(attr.name(), "y2_rect") == 0)
                {
                    zoneMode = ZoneMode::RECTANGLE;
                    ygrid2rect = units::length::meter_t(attr.as_double());
                }
                else if (strcmp(attr.name(), "taleOption") == 0)
                {
                    auto itr = DragonTale::stringToSTATE_NAMESEnumMap.find(attr.value());
                    if (config != nullptr && config->GetMechanism(MechanismTypes::DRAGON_TALE) != nullptr)
                    {
                        if (itr != DragonTale::stringToSTATE_NAMESEnumMap.end())
                        {
                            taleChosenOption = itr->second;
                            isTaleStateChanging = true;
                        }
                        else
                        {
                            hasError = true;
                        }
                    }
                }

                else if (strcmp(attr.name(), "chassisOption") == 0)
                {
                    auto itr = xmlStringToChassisOptionEnumMap.find(attr.value());
                    if (itr != xmlStringToChassisOptionEnumMap.end())
                    {
                        chassisChosenOption = itr->second;
                    }
                    else
                    {
                        hasError = true;
                    }
                }

                else if (strcmp(attr.name(), "headingOption") == 0)
                {
                    auto itr = xmlStringToHeadingOptionEnumMap.find(attr.value());
                    if (itr != xmlStringToHeadingOptionEnumMap.end())
                    {
                        chosenHeadingOption = itr->second;
                    }
                    else
                    {
                        hasError = true;
                    }
                }
                else if (strcmp(attr.name(), "pathUpdateOption") == 0)
                {
                    auto itr = xmlStringToPathUpdateOptionMap.find(attr.value());
                    if (itr != xmlStringToPathUpdateOptionMap.end())
                    {
                        chosenUpdateOption = itr->second;
                        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "ZoneParser", "Update Option Parsed", chosenUpdateOption);
                    }
                    else
                    {
                        hasError = true;
                    }
                }
                else if (strcmp(attr.name(), "avoidOption") == 0)
                {
                    auto itr = xmlStringToAvoidOptionEnumMap.find(attr.value());
                    if (itr != xmlStringToAvoidOptionEnumMap.end())
                    {
                        avoidChosenOption = itr->second;
                    }
                    else
                    {
                        hasError = true;
                    }
                }
            }

            if (!hasError) // if no error returns the zone parameters
            {

                auto circlePose2d = frc::Pose2d(units::length::meter_t(circleX), units::length::meter_t(circleY), units::degree_t(0));
                return (new ZoneParams(circlePose2d,
                                       units::inch_t(radius),
                                       xgrid1rect,
                                       xgrid2rect,
                                       ygrid1rect,
                                       ygrid2rect,
                                       isTaleStateChanging,
                                       taleChosenOption,
                                       chassisChosenOption,
                                       chosenHeadingOption,
                                       chosenUpdateOption,
                                       avoidChosenOption,
                                       zoneMode));
            }

            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("ZoneParser"), string("ParseXML"), string("Has Error"));
        }
    }
    return nullptr; // if error, return nullptr
}
