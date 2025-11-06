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
#include <memory>
#include <string>
#include <vector>

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

// namespace
// {
// 	std::vector<std::unique_ptr<DragonVisionStruct>> ProcessOutputOption(
// 		VisionTargetOption option,
// 		std::vector<std::unique_ptr<DragonVisionStruct>> &targets);
// } // namespace

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
std::vector<std::unique_ptr<DragonVisionStruct>> DragonVision::GetAprilTagVisionTargetInfo(VisionTargetOption option,
																						   const std::vector<FieldAprilTagIDs> &validAprilTagIDs) const

{

	std::vector<std::unique_ptr<DragonVisionStruct>> targets;
	auto cameras = GetCameras(DRAGON_LIMELIGHT_CAMERA_USAGE::APRIL_TAGS);
	if (!cameras.empty())
	{
		return cameras[0]->GetAprilTagVisionTargetInfo(validAprilTagIDs);
	}
	// for (auto cam : cameras)
	// {
	// 	auto camTargets = cam->GetAprilTagVisionTargetInfo(validAprilTagIDs);
	// 	targets.insert((targets.end(), std::make_move_iterator(camTargets.begin()), std::make_move_iterator(camTargets.end())));
	// }

	// return ProcessOutputOption(option, targets);
	return {};
}

std::vector<std::unique_ptr<DragonVisionStruct>> DragonVision::GetObjectDetectionTargetInfo(VisionTargetOption option,
																							const std::vector<int> &validClasses) const
{
	std::vector<std::unique_ptr<DragonVisionStruct>> targets;
	auto cameras = GetCameras(DRAGON_LIMELIGHT_CAMERA_USAGE::OBJECT_DETECTION_ALGAE);
	if (!cameras.empty())
	{
		return cameras[0]->GetObjectDetectionTargetInfo(validClasses);
	}
	// for (auto cam : cameras)
	// {
	// 	auto camTargets = cam->GetObjectDetectionTargetInfo(validClasses);
	// 	targets.insert((targets.end(), std::make_move_iterator(camTargets.begin()), std::make_move_iterator(camTargets.end())));
	// }
	// return ProcessOutputOption(option, targets);
	return {};
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

void DragonVision::Periodic()
{
	// Get the Estimated Robot Pose for each camera
}

// namespace
// {
// 	std::vector<std::unique_ptr<DragonVisionStruct>> ProcessOutputOption(
// 		VisionTargetOption option,
// 		std::vector<std::unique_ptr<DragonVisionStruct>> &targets)
// 	{

// 		switch (option)
// 		{
// 		case VisionTargetOption::CLOSEST_VALID_TARGET:
// 		{
// 			if (!targets.empty())
// 			{
// 				auto closestTargetIt = std::min_element(
// 					targets.begin(),
// 					targets.end(),
// 					[](const std::unique_ptr<DragonVisionStruct> &a, const std::unique_ptr<DragonVisionStruct> &b)
// 					{
// 						return a->targetAreaPercent > b->targetAreaPercent;
// 					});
// 				std::vector<std::unique_ptr<DragonVisionStruct>> closestTarget;
// 				closestTarget.emplace_back(std::move(*closestTargetIt));
// 				return closestTarget;
// 			}
// 			break;
// 		}
// 		case VisionTargetOption::FUSED_TARGET_INFO:
// 		{
// 			// TODO Fusion logic
// 			break;
// 		}
// 		default:
// 			break;
// 		}
// 		return targets;
// 	}
// } // namespace
