
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

#pragma once

// C++ Includes
#include <string>

// Team 302 includes
#include "mechanisms/MechanismTypes.h"
#include "mechanisms/controllers/ControlData.h"

// Forward Declares
class StateMgr;

///	 @class Mech
///  @brief	base mechanism class
class BaseMech
{
public:
    /// @brief create the general mechanism
    /// @param [in] MechanismTypes::MECHANISM_TYPE the type of mechansim
    /// @param [in] std::string the name of the file that will set control parameters for this mechanism
    /// @param [in] std::string the name of the network table for logging information
    BaseMech(MechanismTypes::MECHANISM_TYPE type,
             std::string networkTableName);

    BaseMech(MechanismTypes::MECHANISM_TYPE type);

    /// @brief          Indicates the type of mechanism this is
    /// @return         MechanismTypes::MECHANISM_TYPE
    virtual MechanismTypes::MECHANISM_TYPE GetType() const;

    /// @brief indicate the Network Table name used to setting tracking parameters
    /// @return std::string the name of the network table
    virtual std::string GetNetworkTableName() const;

    /// @brief log data to the network table if it is activated and time period has past
    void LogInformation();

    void ReadConstants(std::string configfilename, int robotId);

    virtual ~BaseMech() = default;

    void SetNetworkFileName(std::string ntName) { m_ntName = ntName; }

protected:
    virtual ControlData *GetControlData(std::string name);

private:
    BaseMech() = delete;

    MechanismTypes::MECHANISM_TYPE m_type;
    std::string m_ntName;
};