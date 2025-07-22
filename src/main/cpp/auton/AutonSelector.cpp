
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

// co-Author: notcharlie, creator of dumb code / copy paster of better code

// Includes
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <sys/stat.h>

#ifdef __linux
#include <dirent.h>
#endif

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Filesystem.h>
#include "networktables/NetworkTableInstance.h"

// Team302 includes
#include "auton/AutonSelector.h"
#include "utils/logging/debug/Logger.h"
#include "utils/FMSData.h"

#include <pugixml/pugixml.hpp>

using namespace std;
using frc::DriverStation;

//---------------------------------------------------------------------
// Method: 		<<constructor>>
// Description: This creates this object and reads the auto script (CSV)
//  			files and displays a list on the dashboard.
//---------------------------------------------------------------------
AutonSelector::AutonSelector()
{
	PutChoicesOnDashboard();
}

string AutonSelector::GetSelectedAutoFile()
{
	std::string autonfile(frc::filesystem::GetDeployDirectory());
	autonfile += std::filesystem::path("/auton/").string();
	autonfile += GetDesiredScoringLevel() + "/";
	autonfile += GetAlianceColor();
	autonfile += GetStartPos();
	autonfile += GetTargetFace();
	autonfile += GetTargetGamePiece();
	autonfile += GetDesiredScoringLevel();
	autonfile += std::string(".xml");

	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("auton"), string("file"), autonfile);

	auto table = nt::NetworkTableInstance::GetDefault().GetTable("auton file");

	table.get()->PutString("determined name", autonfile);

	bool fileExists = FileExists(autonfile);
	bool fileValid = FileValid(autonfile);

	table.get()->PutBoolean("File Exists", fileExists);
	table.get()->PutBoolean("File Valid", fileValid);

	if (!fileExists || !fileValid)
	{
		autonfile = frc::filesystem::GetDeployDirectory();
		autonfile += std::filesystem::path("/auton/").string();
		autonfile += GetAlianceColor();
		autonfile += ("DefaultFile.xml");
	}

	table.get()->PutString("actual file", autonfile);

	return autonfile;
}

bool AutonSelector::FileExists(const std::string &name)
{
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}

bool AutonSelector::FileValid(const std::string &name)
{

	pugi::xml_document doc;
	pugi::xml_parse_result result = doc.load_file(name.c_str());
	if (result)
	{
		return true;
	}
	Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("AutonSelector"), string("FileInvalid: Description ") + name, string(result.description()));
	Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("AutonSelector"), string("FileInvalid: Offset ") + name, static_cast<int>(result.offset));
	return false;
}

string AutonSelector::GetAlianceColor()
{
	return (FMSData::GetAllianceColor() == DriverStation::Alliance::kRed) ? std::string("Red") : std::string("Blue");
}

string AutonSelector::GetStartPos()
{
	return m_startposchooser.GetSelected();
}

string AutonSelector::GetTargetGamePiece()
{
	return m_targetGamePiece.GetSelected();
}

string AutonSelector::GetDesiredScoringLevel()
{
	return m_desiredScoringLevel.GetSelected();
}
string AutonSelector::GetTargetFace()
{
	return m_targetFace.GetSelected();
}

//---------------------------------------------------------------------
// Method: 		PutChoicesOnDashboard
// Description: This puts the list of files in the m_csvFiles attribute
//				up on the dashboard for selection.
// Returns:		void
//---------------------------------------------------------------------
void AutonSelector::PutChoicesOnDashboard()
{
	// Starting Position
	m_startposchooser.AddOption("Left", "Left");
	m_startposchooser.AddOption("Center", "Center");
	m_startposchooser.AddOption("Right", "Right");
	m_startposchooser.SetDefaultOption("Center", "Center");
	frc::SmartDashboard::PutData("StartPos", &m_startposchooser);

	// Game Piece Option
	m_targetGamePiece.AddOption("Coral", "Coral");
	m_targetGamePiece.AddOption("Algae", "Algae");
	m_targetGamePiece.SetDefaultOption("Coral", "Coral");
	frc::SmartDashboard::PutData("Target Game Piece", &m_targetGamePiece);

	// Target Face Option
	m_targetFace.AddOption("A-D", "AD");
	m_targetFace.AddOption("C-F", "CF");
	m_targetFace.AddOption("E-H", "EH");
	m_targetFace.AddOption("G-J", "GJ");
	m_targetFace.AddOption("I-L", "IL");
	m_targetFace.AddOption("K-B", "KB");
	m_targetFace.SetDefaultOption("G-H", "GH");
	frc::SmartDashboard::PutData("Target Face", &m_targetFace);

	// Level Option
	m_desiredScoringLevel.AddOption("L1", "L1");
	m_desiredScoringLevel.AddOption("L2", "L2");
	m_desiredScoringLevel.AddOption("L3", "L3");
	m_desiredScoringLevel.AddOption("L4", "L4");
	m_desiredScoringLevel.SetDefaultOption("L4", "L4");
	frc::SmartDashboard::PutData("Desired Level", &m_desiredScoringLevel);
}
