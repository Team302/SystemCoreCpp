//====================================================================================================================================================
/// Copyright 2022 Lake Orion Robotics FIRST Team 302
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

// C++ Includes
#include <string>

// FRC Includes
#include "frc/Timer.h"

// Team 302 includes
#include "chassis/ChassisConfigMgr.h"
#include "utils/DragonField.h"
#include "utils/FMSData.h"
#include "utils/logging/debug/Logger.h"
#include "vision/DragonLimelight.h"
#include "vision/DragonVision.h"

// Third Party Includes
#include "Limelight/LimelightHelpers.h"

DragonVision *DragonVision::m_dragonVision = nullptr;
DragonVision *DragonVision::GetDragonVision()
{
	if (DragonVision::m_dragonVision == nullptr)
	{
		DragonVision::m_dragonVision = new DragonVision();
	}
	return DragonVision::m_dragonVision;
}

bool DragonVision::HealthCheck(DRAGON_LIMELIGHT_CAMERA_USAGE usage)
{
	bool isHealthy = false;
	auto cameras = GetCameras(usage);
	for (auto cam : cameras)
	{
		isHealthy = cam->IsLimelightRunning();
		if (!isHealthy)
		{
			return isHealthy;
		}
	}
	return isHealthy;
}

bool DragonVision::HealthCheck(DRAGON_LIMELIGHT_CAMERA_IDENTIFIER identifier)
{
	auto camera = GetCameras(identifier);
	if (camera != nullptr)
	{
		return camera->IsLimelightRunning();
	}
	return false;
}

frc::AprilTagFieldLayout DragonVision::m_aprilTagLayout = frc::AprilTagFieldLayout();
frc::AprilTagFieldLayout DragonVision::GetAprilTagLayout()
{
	if (DragonVision::m_aprilTagLayout != frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025ReefscapeWelded))
	{
		DragonVision::m_aprilTagLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025ReefscapeWelded);
	}
	return DragonVision::m_aprilTagLayout;
}

DragonVision::DragonVision()
{
}

void DragonVision::AddLimelight(DragonLimelight *camera, DRAGON_LIMELIGHT_CAMERA_USAGE usage)
{
	m_dragonLimelightMap.insert(std::pair<DRAGON_LIMELIGHT_CAMERA_USAGE, DragonLimelight *>(usage, camera));
}

std::vector<int> DragonVision::GetReefTags(frc::DriverStation::Alliance allianceColor) const
{
	std::vector<int> tagIdsToCheck = {};
	if (allianceColor == frc::DriverStation::Alliance::kBlue)
	{
		tagIdsToCheck.emplace_back(17);
		tagIdsToCheck.emplace_back(18);
		tagIdsToCheck.emplace_back(19);
		tagIdsToCheck.emplace_back(20);
		tagIdsToCheck.emplace_back(21);
		tagIdsToCheck.emplace_back(22);
	}
	else
	{
		tagIdsToCheck.emplace_back(6);
		tagIdsToCheck.emplace_back(7);
		tagIdsToCheck.emplace_back(8);
		tagIdsToCheck.emplace_back(9);
		tagIdsToCheck.emplace_back(10);
		tagIdsToCheck.emplace_back(11);
	}
	return tagIdsToCheck;
}
std::vector<int> DragonVision::GetCoralStationsTags(frc::DriverStation::Alliance allianceColor) const
{
	std::vector<int> tagIdsToCheck = {};
	if (allianceColor == frc::DriverStation::Alliance::kBlue)
	{
		tagIdsToCheck.emplace_back(12);
		tagIdsToCheck.emplace_back(13);
	}
	else
	{
		tagIdsToCheck.emplace_back(1);
		tagIdsToCheck.emplace_back(2);
	}
	return tagIdsToCheck;
}
std::vector<int> DragonVision::GetProcessorTags(frc::DriverStation::Alliance allianceColor) const
{
	std::vector<int> tagIdsToCheck = {};
	if (allianceColor == frc::DriverStation::Alliance::kBlue)
	{
		tagIdsToCheck.emplace_back(16);
	}
	else
	{
		tagIdsToCheck.emplace_back(3);
	}
	return tagIdsToCheck;
}
std::vector<int> DragonVision::GetBargeTags(frc::DriverStation::Alliance allianceColor) const
{
	std::vector<int> tagIdsToCheck = {};
	if (allianceColor == frc::DriverStation::Alliance::kBlue)
	{
		tagIdsToCheck.emplace_back(4);
		tagIdsToCheck.emplace_back(14);
	}
	else
	{
		tagIdsToCheck.emplace_back(5);
		tagIdsToCheck.emplace_back(15);
	}
	return tagIdsToCheck;
}

std::vector<DragonLimelight *> DragonVision::GetCameras(DRAGON_LIMELIGHT_CAMERA_USAGE usage) const
{
	std::vector<DragonLimelight *> validCameras;
	for (auto it = m_dragonLimelightMap.begin(); it != m_dragonLimelightMap.end(); ++it)
	{
		bool addCam = false;
		auto cam = (*it).second;
		if (usage == DRAGON_LIMELIGHT_CAMERA_USAGE::ALGAE_AND_APRIL_TAGS)
		{
			if (cam->IsLimelightRunning())
			{
				validCameras.emplace_back(cam);
			}
		}
		else
		{

			addCam = (*it).first == usage;
			if (!addCam)
			{
				if ((*it).first == DRAGON_LIMELIGHT_CAMERA_USAGE::ALGAE_AND_APRIL_TAGS)
				{
					// auto pipe = cam->GetPipeline();  TODO update when using visionstruct
					auto pipe = DRAGON_LIMELIGHT_PIPELINE::APRIL_TAG;
					if (usage == DRAGON_LIMELIGHT_CAMERA_USAGE::APRIL_TAGS)
					{
						addCam = pipe == DRAGON_LIMELIGHT_PIPELINE::APRIL_TAG;
					}
					else if (usage == DRAGON_LIMELIGHT_CAMERA_USAGE::OBJECT_DETECTION_ALGAE)
					{
						addCam = pipe == DRAGON_LIMELIGHT_PIPELINE::MACHINE_LEARNING_PL || pipe == DRAGON_LIMELIGHT_PIPELINE::COLOR_THRESHOLD;
					}
				}
			}
		}

		if (addCam)
		{
			if (cam->IsLimelightRunning())
			{
				validCameras.emplace_back(cam);
			}
		}
	}
	return validCameras;
}

DragonLimelight *DragonVision::GetCameras(DRAGON_LIMELIGHT_CAMERA_IDENTIFIER identifier) const
{
	auto cameras = GetCameras(DRAGON_LIMELIGHT_CAMERA_USAGE::ALGAE_AND_APRIL_TAGS);
	for (auto cam : cameras)
	{
		if (cam->GetCameraIdentifier() == identifier)
		{
			return cam;
		}
		return nullptr;
	}
	return nullptr;
}

void DragonVision::SetPipeline(DRAGON_LIMELIGHT_CAMERA_USAGE position, DRAGON_LIMELIGHT_PIPELINE pipeline)
{
	auto cameras = GetCameras(position);
	for (auto cam : cameras)
	{
		// if (cam->GetPipeline() != pipeline)
		// {
		cam->SetPipeline(pipeline);
		// }
	}
}