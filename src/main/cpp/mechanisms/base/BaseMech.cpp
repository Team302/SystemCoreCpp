//====================================================================================================================================================
// Copyright 2024 Lake Orion Robotics FIRST Team 302
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
#include <iostream>
#include <memory>
#include <string>

// FRC includes
#include "frc/Filesystem.h"

// Team 302 includes
#include "mechanisms/base/BaseMech.h"
#include "mechanisms/MechanismTypes.h"
#include "utils/logging/debug/Logger.h"

// Third Party Includes
#include "units/time.h"
#include "pugixml/pugixml.hpp"

using namespace std;

/// @brief create the general mechanism
/// @param [in] MechanismTypes::MECHANISM_TYPE the type of mechansim
/// @param [in] std::string the name of the file that will set control parameters for this mechanism
/// @param [in] std::string the name of the network table for logging information
BaseMech::BaseMech(MechanismTypes::MECHANISM_TYPE type,
                   string networkTableName) : m_type(type),
                                              m_ntName(networkTableName)
{
    if (networkTableName.empty())
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("Mech"), string("Mech"), string("network table name is not specified"));
    }
}
BaseMech::BaseMech(MechanismTypes::MECHANISM_TYPE type) : m_type(type),
                                                          m_ntName()
{
}

/// @brief          Indicates the type of mechanism this is
/// @return         MechanismTypes::MECHANISM_TYPE
MechanismTypes::MECHANISM_TYPE BaseMech::GetType() const
{
    return m_type;
}

/// @brief indicate the network table name used to for logging parameters
/// @return std::string the name of the network table
string BaseMech::GetNetworkTableName() const
{
    return m_ntName;
}

/// @brief log data to the network table if it is activated and time period has past
void BaseMech::LogInformation()
{
    // NO-OP - subclasses override when necessary
}

ControlData *BaseMech::GetControlData(string name)
{
    return nullptr;
}

void BaseMech::ReadConstants(string configfilename, int robotId)
{
    auto deployDir = frc::filesystem::GetDeployDirectory();
    auto filename = deployDir + string("/") + std::to_string(robotId) + string("/mechanisms/") + configfilename;

    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("mech"), string("Reading parameters from "), filename);

    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(filename.c_str());

    if (result)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("mech"), string(""), "Loaded xml file");

        pugi::xml_node parent = doc.root();

        pugi::xml_node MechanismParameters = parent.child("MechanismParameters");
        pugi::xml_node controlDataNodes = MechanismParameters.child("MotorControlData");

        for (pugi::xml_node aNode = controlDataNodes.child("MinimalMotorControlData"); aNode; aNode = aNode.next_sibling("MinimalMotorControlData"))
        {
            string name = aNode.attribute("name").as_string();

            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("mech"), string("Loaded ControlData "), name);

            ControlData *theData = GetControlData(name);
            if (theData != nullptr)
            {
                theData->SetP(aNode.attribute("pGain").as_double());
                theData->SetI(aNode.attribute("iGain").as_double());
                theData->SetD(aNode.attribute("dGain").as_double());
                theData->SetF(aNode.attribute("fGain").as_double());
                theData->SetS(aNode.attribute("sGain").as_double());
                theData->SetA(aNode.attribute("aGain").as_double());
                theData->SetV(aNode.attribute("vGain").as_double());
                theData->SetIZone(aNode.attribute("iZone").as_double());
                theData->SetPeakValue(aNode.attribute("peakValue").as_double());
                theData->SetNominalValue(aNode.attribute("nominalValue").as_double());
                theData->SetMaxAcceleration(aNode.attribute("maxAcceleration").as_double());
                theData->SetCruiseVelocity(aNode.attribute("cruiseVelocity").as_double());
                theData->SetFOCEnabled(aNode.attribute("enableFOC").as_bool());

                if (strcmp(aNode.attribute("feedForwardType").as_string(), "VOLTAGE") == 0)
                    theData->SetFType(ControlData::FEEDFORWARD_TYPE::VOLTAGE);
                else if (strcmp(aNode.attribute("feedForwardType").as_string(), "TORQUE_CURRENT") == 0)
                    theData->SetFType(ControlData::FEEDFORWARD_TYPE::TORQUE_CURRENT);
                else
                    theData->SetFType(ControlData::FEEDFORWARD_TYPE::DUTY_CYCLE);

                if (strcmp(aNode.attribute("controlType").as_string(), "VOLTAGE_OUTPUT") == 0)
                    theData->SetMode(ControlModes::CONTROL_TYPE::VOLTAGE_OUTPUT);
                else if (strcmp(aNode.attribute("controlType").as_string(), "POSITION_DEGREES") == 0)
                    theData->SetMode(ControlModes::CONTROL_TYPE::POSITION_DEGREES);
                else if (strcmp(aNode.attribute("controlType").as_string(), "POSITION_INCH") == 0)
                    theData->SetMode(ControlModes::CONTROL_TYPE::POSITION_INCH);
                else
                    theData->SetMode(ControlModes::CONTROL_TYPE::PERCENT_OUTPUT);
            }
            else
            {
                std::cout << "Did not find the xml node for COntrolData " << name << std::endl;
            }
        }
    }
    else
    {
        std::cout << "Config File not found" << std::endl;

        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("mech"), string("Config File not found"), configfilename);
    }
}
