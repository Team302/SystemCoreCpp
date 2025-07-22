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
#include <memory>
#include <string>
#include <utility>

// #include <vector>

// FRC includes

// Team 302 includes

// Third Party Includes
#include <string>
#include <frc/GenericHID.h>
#include <gamepad/IDragonGamepad.h>
#include <gamepad/DragonXBox.h>
#include <gamepad/DragonGamepad.h>
#include "teleopcontrol/TeleopControl.h"
#include <teleopcontrol/TeleopControlFunctions.h>
#include <teleopcontrol/TeleopControlMap.h>
#include <frc/DriverStation.h>
#include "utils/logging/debug/Logger.h"

using frc::DriverStation;
using frc::GenericHID;
using std::make_pair;
using std::pair;
using std::string;
using std::vector;

//----------------------------------------------------------------------------------
// Method:      GetInstance
// Description: If there isn't an instance of this class, it will create one.  The
//              single class instance will be returned.
// Returns:     OperatorInterface*  instance of this class
//----------------------------------------------------------------------------------
TeleopControl *TeleopControl::m_instance = nullptr; // initialize the instance variable to nullptr
TeleopControl *TeleopControl::GetInstance()
{
	if (TeleopControl::m_instance == nullptr)
	{
		TeleopControl::m_instance = new TeleopControl();
	}
	if (TeleopControl::m_instance != nullptr && !TeleopControl::m_instance->IsInitialized())
	{
		TeleopControl::m_instance->Initialize();
	}
	return TeleopControl::m_instance;
}
//----------------------------------------------------------------------------------
// Method:      OperatorInterface <<constructor>>
// Description: This will construct and initialize the object.
//              It maps the functions to the buttons/axis.
//---------------------------------------------------------------------------------
TeleopControl::TeleopControl() : m_controller(),
								 m_numControllers(0)

{
	for (auto i = 0; i < DriverStation::kJoystickPorts; ++i)
	{
		m_controller[i] = nullptr;
	}
	Initialize();
}

bool TeleopControl::IsInitialized() const
{
	return m_numControllers > 0;
}
void TeleopControl::Initialize()
{
	InitializeControllers();
}

void TeleopControl::InitializeControllers()
{
	for (int inx = 0; inx < DriverStation::kJoystickPorts; ++inx)
	{
		InitializeController(inx);
	}
}

void TeleopControl::InitializeController(int port)
{
	if (m_controller[port] == nullptr)
	{
		if (port == 0) // Special handling for port 0
		{
			m_hybridController = new DragonHybridController(port);
			m_controller[port] = m_hybridController->GetNonCommandController();
		}
		else if (DriverStation::GetJoystickIsGamepad(port))
		{
			auto xbox = new DragonXBox(port);
			m_controller[port] = xbox;
		}
		else if (DriverStation::GetJoystickType(port) == GenericHID::kHID1stPerson)
		{
			auto gamepad = new DragonGamepad(port);
			m_controller[port] = gamepad;
		}

		if (m_controller[port] != nullptr)
		{
			InitializeAxes(port);
			InitializeButtons(port);
		}
	}
}

DragonHybridController *TeleopControl::GetHybridController()
{
	return m_hybridController;
}
void TeleopControl::InitializeAxes(int port)
{
	if (m_controller[port] != nullptr)
	{
		auto functions = GetAxisFunctionsOnController(port);
		for (auto function : functions)
		{
			auto itr = teleopControlMapAxisMap.find(function);
			if (itr != teleopControlMapAxisMap.end())
			{
				auto axisInfo = itr->second;
				m_controller[port]->SetAxisDeadband(axisInfo.axisId, axisInfo.deadbandType);
				m_controller[port]->SetAxisProfile(axisInfo.axisId, axisInfo.profile);
				m_controller[port]->SetAxisScale(axisInfo.axisId, axisInfo.scaleFactor);
				m_controller[port]->SetAxisFlipped(axisInfo.axisId, axisInfo.direction != TeleopControlMappingEnums::AXIS_DIRECTION::SYNCED);
			}
		}
	}
}

void TeleopControl::InitializeButtons(int port)
{
	if (m_controller[port] != nullptr)
	{
		auto functions = GetButtonFunctionsOnController(port);
		for (auto function : functions)
		{
			auto itr = teleopControlMapButtonMap.find(function);
			if (itr != teleopControlMapButtonMap.end())
			{
				auto buttonInfo = itr->second;
				if (buttonInfo.mode != TeleopControlMappingEnums::BUTTON_MODE::STANDARD)
				{
					m_controller[port]->SetButtonMode(buttonInfo.buttonId, buttonInfo.mode);
				}
			}
		}
	}
}
vector<TeleopControlFunctions::FUNCTION> TeleopControl::GetAxisFunctionsOnController(int controller)
{
	vector<TeleopControlFunctions::FUNCTION> functions;

	for (auto itr = teleopControlMapAxisMap.begin(); itr != teleopControlMapAxisMap.end(); ++itr)
	{
		if (itr->second.controllerNumber == controller)
		{
			functions.emplace_back(itr->first);
		}
	}
	return functions;
}

vector<TeleopControlFunctions::FUNCTION> TeleopControl::GetButtonFunctionsOnController(int controller)
{
	vector<TeleopControlFunctions::FUNCTION> functions;

	for (auto itr = teleopControlMapButtonMap.begin(); itr != teleopControlMapButtonMap.end(); ++itr)
	{
		if (itr->second.controllerNumber == controller)
		{
			functions.emplace_back(itr->first);
		}
	}
	return functions;
}

pair<IDragonGamepad *, TeleopControlMappingEnums::AXIS_IDENTIFIER> TeleopControl::GetAxisInfo(
	TeleopControlFunctions::FUNCTION function // <I> - controller with this function
)
{
	IDragonGamepad *controller = nullptr;
	TeleopControlMappingEnums::AXIS_IDENTIFIER axis = TeleopControlMappingEnums::AXIS_IDENTIFIER::UNDEFINED_AXIS;

	if (!IsInitialized())
	{
		Initialize();
	}

	auto itr = teleopControlMapAxisMap.find(function);
	if (itr != teleopControlMapAxisMap.end())
	{
		auto axisInfo = itr->second;
		if (m_controller[axisInfo.controllerNumber] != nullptr)
		{
			controller = m_controller[axisInfo.controllerNumber];
			axis = axisInfo.axisId;
		}
	}
	return make_pair(controller, axis);
}

pair<IDragonGamepad *, TeleopControlMappingEnums::BUTTON_IDENTIFIER> TeleopControl::GetButtonInfo(
	TeleopControlFunctions::FUNCTION function // <I> - controller with this function
)
{
	IDragonGamepad *controller = nullptr;
	TeleopControlMappingEnums::BUTTON_IDENTIFIER btn = TeleopControlMappingEnums::UNDEFINED_BUTTON;

	if (!IsInitialized())
	{
		Initialize();
	}

	auto itr = teleopControlMapButtonMap.find(function);
	if (itr != teleopControlMapButtonMap.end())
	{
		auto buttonInfo = itr->second;
		if (m_controller[buttonInfo.controllerNumber] != nullptr)
		{
			controller = m_controller[buttonInfo.controllerNumber];
			btn = buttonInfo.buttonId;
		}
	}
	return make_pair(controller, btn);
}

//------------------------------------------------------------------
// Method:      SetAxisScaleFactor
// Description: Allow the range of values to be set smaller than
//              -1.0 to 1.0.  By providing a scale factor between 0.0
//              and 1.0, the range can be made smaller.  If a value
//              outside the range is provided, then the value will
//              be set to the closest bounding value (e.g. 1.5 will
//              become 1.0)
// Returns:     void
//------------------------------------------------------------------
void TeleopControl::SetAxisScaleFactor(
	TeleopControlFunctions::FUNCTION function, // <I> - function that will update an axis
	double scaleFactor						   // <I> - scale factor used to limit the range
)
{
	auto info = GetAxisInfo(function);
	if (info.first != nullptr && info.second != TeleopControlMappingEnums::AXIS_IDENTIFIER::UNDEFINED_AXIS)
	{
		info.first->SetAxisScale(info.second, scaleFactor);
	}
}

void TeleopControl::SetDeadBand(
	TeleopControlFunctions::FUNCTION function,
	TeleopControlMappingEnums::AXIS_DEADBAND deadband)
{
	auto info = GetAxisInfo(function);
	if (info.first != nullptr && info.second != TeleopControlMappingEnums::AXIS_IDENTIFIER::UNDEFINED_AXIS)
	{
		info.first->SetAxisDeadband(info.second, deadband);
	}
}

//------------------------------------------------------------------
// Method:      SetAxisProfile
// Description: Sets the axis profile for the specifed axis
// Returns:     void
//------------------------------------------------------------------
void TeleopControl::SetAxisProfile(
	TeleopControlFunctions::FUNCTION function,		// <I> - function that will update an axis
	TeleopControlMappingEnums::AXIS_PROFILE profile // <I> - profile to use
)
{
	auto info = GetAxisInfo(function);
	if (info.first != nullptr && info.second != TeleopControlMappingEnums::AXIS_IDENTIFIER::UNDEFINED_AXIS)
	{
		info.first->SetAxisProfile(info.second, profile);
	}
}

//------------------------------------------------------------------
// Method:      GetAxisValue
// Description: Reads the joystick axis, removes any deadband (small
//              value) and then scales as requested.
// Returns:     double   -  scaled axis value
//------------------------------------------------------------------
double TeleopControl::GetAxisValue(
	TeleopControlFunctions::FUNCTION function // <I> - function that whose axis will be read
)
{
	double value = 0.0;
	auto info = GetAxisInfo(function);
	if (info.first != nullptr && info.second != TeleopControlMappingEnums::AXIS_IDENTIFIER::UNDEFINED_AXIS)
	{
		value = info.first->GetAxisValue(info.second);
	}
	return value;
}

//------------------------------------------------------------------
// Method:      IsButtonPressed
// Description: Reads the button value.  Also allows POV, bumpers,
//              and triggers to be treated as buttons.
// Returns:     bool   -  scaled axis value
//------------------------------------------------------------------
bool TeleopControl::IsButtonPressed(
	TeleopControlFunctions::FUNCTION function // <I> - function that whose button will be read
)
{
	bool isSelected = false;
	auto info = GetButtonInfo(function);
	if (info.first != nullptr && info.second != TeleopControlMappingEnums::UNDEFINED_BUTTON)
	{
		isSelected = info.first->IsButtonPressed(info.second);
	}
	return isSelected;
}

void TeleopControl::SetRumble(
	TeleopControlFunctions::FUNCTION function, // <I> - controller with this function
	bool leftRumble,						   // <I> - rumble left
	bool rightRumble						   // <I> - rumble right
)
{

	auto info = GetButtonInfo(function);
	if (info.first != nullptr)
	{
		info.first->SetRumble(leftRumble, rightRumble);
	}
	else
	{
		auto info2 = GetAxisInfo(function);
		if (info2.first != nullptr)
		{
			info2.first->SetRumble(leftRumble, rightRumble);
		}
	}
}

void TeleopControl::SetRumble(
	int controller,	 // <I> - controller to rumble
	bool leftRumble, // <I> - rumble left
	bool rightRumble // <I> - rumble right
)
{
	if (m_controller[controller] != nullptr)
	{
		m_controller[controller]->SetRumble(leftRumble, rightRumble);
	}
}

void TeleopControl::LogInformation()
{
	auto self = const_cast<TeleopControl *>(this);
	for (int inx = 0; inx < DriverStation::kJoystickPorts; ++inx)
	{
		if (m_controller[inx] != nullptr)
		{
			auto functions = self->GetAxisFunctionsOnController(inx);
			for (auto function : functions)
			{
				Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("TeleopControl-axis"), std::to_string(function), self->GetAxisValue(function));
			}

			functions.clear();
			functions = self->GetButtonFunctionsOnController(inx);
			for (auto function : functions)
			{
				Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("TeleopControl-button"), std::to_string(function), self->IsButtonPressed(function));
			}
		}
	}
}

frc2::Trigger TeleopControl::GetCommandTrigger(TeleopControlFunctions::FUNCTION function)
{
	// Find the button mapping for the given function
	auto itr = teleopControlMapButtonMap.find(function);
	auto buttonInfo = itr->second;

	auto controller = m_hybridController->GetCommandController();
	if (controller != nullptr)
	{

		// Map the button identifier to the corresponding CommandXboxController method
		switch (buttonInfo.buttonId)
		{
		case TeleopControlMappingEnums::A_BUTTON:
			return controller->A();
		case TeleopControlMappingEnums::B_BUTTON:
			return controller->B();
		case TeleopControlMappingEnums::X_BUTTON:
			return controller->X();
		case TeleopControlMappingEnums::Y_BUTTON:
			return controller->Y();
		case TeleopControlMappingEnums::LEFT_BUMPER:
			return controller->LeftBumper();
		case TeleopControlMappingEnums::RIGHT_BUMPER:
			return controller->RightBumper();
		case TeleopControlMappingEnums::SELECT_BUTTON:
			return controller->Back(); // 'Select' is usually 'Back' in FRC
		case TeleopControlMappingEnums::START_BUTTON:
			return controller->Start();
		case TeleopControlMappingEnums::LEFT_STICK_PRESSED:
			return controller->LeftStick();
		case TeleopControlMappingEnums::RIGHT_STICK_PRESSED:
			return controller->RightStick();
		case TeleopControlMappingEnums::LEFT_TRIGGER_PRESSED:
			return controller->LeftTrigger();
		case TeleopControlMappingEnums::RIGHT_TRIGGER_PRESSED:
			return controller->RightTrigger();
		case TeleopControlMappingEnums::POV_0:
			return controller->POVUp();
		case TeleopControlMappingEnums::POV_90:
			return controller->POVRight();
		case TeleopControlMappingEnums::POV_180:
			return controller->POVDown();
		case TeleopControlMappingEnums::POV_270:
			return controller->POVLeft();
			// NOTE: CommandXboxController does not have direct support for diagonal POV directions.
			// You would need to use `controller->GetPOV()` and a lambda for those, e.g.:
			// return frc2::Trigger([controller] { return controller->GetPOV() == 45; });
			// For simplicity, this implementation only includes cardinal directions. TODO: implement the comment above

		default:
			Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("TeleopControl-Command"), std::to_string(function), "Couldn't map the TeleopControlMapEnum");
		}
	}
	else
	{
		Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("TeleopControl-Command"), std::to_string(function), "Controller is null.");
	}
	return frc2::Trigger([]()
						 { return false; }); // Return a trigger that is always inactive if the controller is null or the function is not mapped
}
frc2::Trigger TeleopControl::GetAxisAsTrigger(TeleopControlFunctions::FUNCTION function, double threshold)
{
	return frc2::Trigger([this, function, threshold]
						 { return this->GetAxisValue(function) > threshold; });
}
