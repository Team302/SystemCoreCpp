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

#include "auton/PrimitiveParams.h"
#include "auton/PrimitiveParser.h"
#include "auton/ZoneParams.h"
#include "auton/ZoneParser.h"
#include "chassis/ChassisOptionEnums.h"
#include "frc/Filesystem.h"

#include "utils/logging/debug/Logger.h"

#include <pugixml/pugixml.hpp>

using namespace std;
using namespace pugi;

PrimitiveParamsVector PrimitiveParser::ParseXML(string fulldirfile)
{

    PrimitiveParamsVector paramVector;
    auto hasError = false;

    // initialize the xml string to enum maps
    map<string, PRIMITIVE_IDENTIFIER> primStringToEnumMap;
    primStringToEnumMap["DO_NOTHING"] = DO_NOTHING;
    primStringToEnumMap["HOLD_POSITION"] = HOLD_POSITION;
    primStringToEnumMap["TRAJECTORY_DRIVE"] = TRAJECTORY_DRIVE;
    primStringToEnumMap["RESET_POSITION"] = RESET_POSITION;
    primStringToEnumMap["VISION_ALIGN"] = VISION_ALIGN;
    primStringToEnumMap["DRIVE_STOP_MECH"] = DO_NOTHING_MECHANISMS;

    map<string, ChassisOptionEnums::HeadingOption>
        headingOptionMap;
    headingOptionMap["MAINTAIN"] = ChassisOptionEnums::HeadingOption::MAINTAIN;
    headingOptionMap["IGNORE"] = ChassisOptionEnums::HeadingOption::IGNORE;
    headingOptionMap["SPECIFIED_ANGLE"] = ChassisOptionEnums::HeadingOption::SPECIFIED_ANGLE;
    headingOptionMap["FACE_GAME_PIECE"] = ChassisOptionEnums::HeadingOption::FACE_GAME_PIECE;
    headingOptionMap["FACE_CORAL_STATION"] = ChassisOptionEnums::HeadingOption::FACE_CORAL_STATION;

    map<string, PrimitiveParams::VISION_ALIGNMENT>
        xmlStringToVisionAlignmentEnumMap{
            {"UNKNOWN", PrimitiveParams::VISION_ALIGNMENT::UNKNOWN},
            {"ALGAE", PrimitiveParams::VISION_ALIGNMENT::ALGAE},
            {"CORAL_STATION", PrimitiveParams::VISION_ALIGNMENT::CORAL_STATION},
            {"PROCESSOR", PrimitiveParams::VISION_ALIGNMENT::PROCESSOR}};

    map<string, ChassisOptionEnums::DriveStateType> updateOptionMap{{"RIGHT_REEF_BRANCH", ChassisOptionEnums::DriveStateType::DRIVE_TO_RIGHT_REEF_BRANCH},
                                                                    {"LEFT_REEF_BRANCH", ChassisOptionEnums::DriveStateType::DRIVE_TO_LEFT_REEF_BRANCH},
                                                                    {"CORAL_STATION", ChassisOptionEnums::DriveStateType::DRIVE_TO_CORAL_STATION},
                                                                    {"BARGE", ChassisOptionEnums::DRIVE_TO_BARGE},
                                                                    {"NOTHING", ChassisOptionEnums::DriveStateType::STOP_DRIVE}};

    xml_document doc;
    xml_parse_result result = doc.load_file(fulldirfile.c_str());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PrimitiveParser", "Original File", fulldirfile.c_str());
    if (!result)
    {
        auto deployDir = frc::filesystem::GetDeployDirectory();
        auto autonDir = deployDir + "/auton/";

        string updfulldirfile = autonDir;
        updfulldirfile += fulldirfile;

        result = doc.load_file(updfulldirfile.c_str());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PrimitiveParser", "updated File", updfulldirfile.c_str());
    }

    if (result)
    {
        xml_node auton = doc.root();
        for (xml_node node = auton.first_child(); node; node = node.next_sibling())
        {
            for (xml_node primitiveNode = node.first_child(); primitiveNode; primitiveNode = primitiveNode.next_sibling())
            {
                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PrimitiveParser", "node", primitiveNode.name());

                if (strcmp(primitiveNode.name(), "snippet") == 0)
                {
                    for (xml_attribute attr = primitiveNode.first_attribute(); attr; attr = attr.next_attribute())
                    {
                        if (strcmp(attr.name(), "file") == 0)
                        {
                            auto filename = string("snippets/") + string(attr.value());
                            auto snippetParams = ParseXML(filename);
                            if (!snippetParams.empty())
                            {
                                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("PrimitiveParser"), string("snippet has parms"), (double)snippetParams.size());
                                for (auto snippet : snippetParams)
                                {
                                    paramVector.emplace_back(snippet);
                                }
                            }
                            else
                            {
                                Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("PrimitiveParser"), string("snippet had no params"), filename.c_str());
                                hasError = true;
                            }
                        }
                        else
                        {
                            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("PrimitiveParser"), string("snippet unknown attr"), attr.name());
                            hasError = true;
                        }
                    }
                }
                else if (strcmp(primitiveNode.name(), "primitive") == 0)
                {
                    auto primitiveType = UNKNOWN_PRIMITIVE;
                    units::time::second_t time = units::time::second_t(15.0);
                    auto headingOption = ChassisOptionEnums::HeadingOption::IGNORE;
                    auto heading = 0.0;
                    auto visionAlignment = PrimitiveParams::VISION_ALIGNMENT::UNKNOWN;

                    auto taleState = DragonTale::STATE_READY;
                    bool changeTaleState = false;
                    auto config = MechanismConfigMgr::GetInstance()->GetCurrentConfig();

                    std::string pathName;
                    std::string choreoTrajectoryName;
                    ZoneParamsVector zones;

                    ChassisOptionEnums::DriveStateType pathUpdateOption = ChassisOptionEnums::DriveStateType::STOP_DRIVE;

                    Logger::GetLogger()
                        ->LogData(LOGGER_LEVEL::PRINT, string("PrimitiveParser"), string("About to parse primitive"), (double)paramVector.size());

                    for (xml_attribute attr = primitiveNode.first_attribute(); attr; attr = attr.next_attribute())
                    {
                        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("PrimitiveParser"), string("attr"), attr.name());
                        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("PrimitiveParser"), string("value"), attr.value());
                        if (strcmp(attr.name(), "id") == 0)
                        {
                            auto paramStringToEnumItr = primStringToEnumMap.find(attr.value());
                            if (paramStringToEnumItr != primStringToEnumMap.end())
                            {
                                primitiveType = paramStringToEnumItr->second;
                            }
                            else
                            {
                                Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("PrimitiveParser"), string("ParseXML invalid id"), attr.value());
                                hasError = true;
                            }
                        }

                        else if (strcmp(attr.name(), "time") == 0)
                        {
                            time = units::time::second_t(attr.as_float());
                        }
                        else if (strcmp(attr.name(), "headingOption") == 0)
                        {
                            auto headingItr = headingOptionMap.find(attr.value());
                            if (headingItr != headingOptionMap.end())
                            {
                                headingOption = headingItr->second;
                            }
                            else
                            {
                                Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("PrimitiveParser"), string("ParseXML invalid heading option"), attr.value());
                                hasError = true;
                            }
                        }
                        else if (strcmp(attr.name(), "pathUpdateOption") == 0)
                        {
                            auto updateOptionItr = updateOptionMap.find(attr.value());
                            if (updateOptionItr != updateOptionMap.end())
                            {
                                pathUpdateOption = updateOptionItr->second;
                            }
                            else
                            {
                                Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("PrimitiveParser"), string("ParseXML invalid path update option"), attr.value());
                                hasError = true;
                            }
                        }
                        else if (strcmp(attr.name(), "heading") == 0)
                        {
                            heading = attr.as_float();
                        }
                        else if (strcmp(attr.name(), "choreoname") == 0)
                        {
                            choreoTrajectoryName = attr.value();
                        }
                        else if (strcmp(attr.name(), "taleOption") == 0)
                        {
                            if (config != nullptr && config->GetMechanism(MechanismTypes::DRAGON_TALE) != nullptr)
                            {
                                auto taleStateItr = DragonTale::stringToSTATE_NAMESEnumMap.find(attr.value());
                                if (taleStateItr != DragonTale::stringToSTATE_NAMESEnumMap.end())
                                {
                                    taleState = taleStateItr->second;
                                    changeTaleState = true;
                                }
                            }
                        }

                        else if (strcmp(attr.name(), "visionAlignment") == 0)
                        {
                            /** TODO come back to this
                            auto visionAlignmentItr = xmlStringToVisionAlignmentEnumMap.find(attr.value());
                            if (visionAlignmentItr != xmlStringToVisionAlignmentEnumMap.end())
                            {
                                visionAlignment = visionAlignmentItr->second;
                            }
                            else
                            {
                                Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("PrimitiveParser"), string("ParseXML invalid attribute"), attr.name());
                                hasError = true;
                            }
                            **/
                        }
                    }

                    if (!hasError)
                    {
                        for (xml_node child = primitiveNode.first_child(); child && !hasError; child = child.next_sibling())
                        {

                            for (xml_attribute attr = child.first_attribute(); attr; attr = attr.next_attribute())
                            {
                                if (strcmp(attr.name(), "filename") == 0)
                                {
                                    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "zone filename", "name", std::string(attr.value()));
                                    auto zone = ZoneParser::ParseXML(attr.value());
                                    if (zone != nullptr)
                                    {
                                        zones.emplace_back(zone);
                                    }
                                }
                            }
                        }
                    }

                    if (!hasError)
                    {
                        paramVector.emplace_back(new PrimitiveParams(primitiveType,
                                                                     time,
                                                                     headingOption,
                                                                     heading,
                                                                     choreoTrajectoryName,
                                                                     zones, // vector of all zones included as part of the path
                                                                            // can have multiple zones as part of a complex path
                                                                     visionAlignment,
                                                                     changeTaleState,
                                                                     taleState,
                                                                     pathUpdateOption));
                    }
                    else
                    {
                        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("PrimitiveParser"), string("ParseXML"), string("Has Error"));
                    }
                }
            }
        }
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("PrimitiveParser"), string("ParseXML error message"), result.description());
    }

    Print(paramVector);

    return paramVector;
}

void PrimitiveParser::Print(PrimitiveParamsVector paramVector)
{
    auto slot = 0;
    for (auto param : paramVector)
    {
        string ntName = string("Primitive ") + to_string(slot);
        auto logger = Logger::GetLogger();
        logger->LogData(LOGGER_LEVEL::PRINT, ntName, string("Primitive ID"), to_string(param->GetID()));
        logger->LogData(LOGGER_LEVEL::PRINT, ntName, string("Time"), param->GetTime().to<double>());
        logger->LogData(LOGGER_LEVEL::PRINT, ntName, string("Heading Option"), to_string(param->GetHeadingOption()));
        logger->LogData(LOGGER_LEVEL::PRINT, ntName, string("Heading"), param->GetHeading());
        logger->LogData(LOGGER_LEVEL::PRINT, ntName, string("Choreo Trajectory Name"), param->GetTrajectoryName());
        logger->LogData(LOGGER_LEVEL::PRINT, ntName, string("vision alignment"), param->GetVisionAlignment());
        logger->LogData(LOGGER_LEVEL::PRINT, ntName, string("Dragon Tale State"), param->GetTaleState());
        logger->LogData(LOGGER_LEVEL::PRINT, ntName, string("num zones"), (double)param->GetZones().size());

        slot++;
    }
}