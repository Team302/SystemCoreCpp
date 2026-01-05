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

// Developer documentation:
// Brief: High-level manager for vision subsystems (Limelight + QUEST).
// Responsibilities:
//  - Singleton access via DragonVision::GetDragonVision()
//  - Manage multiple DragonLimelight instances and a DragonQuest instance
//  - Query and aggregate AprilTag and object-detection targets across cameras
//  - Select targets according to VisionTargetOption and return selected target data
//  - Provide robot pose estimates fused/selected from camera poses
//  - Configure per-camera pipelines and set robot pose for vision subsystems
//
// Notes:
//  - Not thread-safe; callers must synchronize if used from multiple threads concurrently.
//  - ProcessOutputOption returns indices into a vector that may be moved-from by callers;
//    callers must be aware of ownership transfer when moving unique_ptrs.
//  - HealthCheck relies on DragonLimelight::IsLimelightRunning() for camera health.
//
// TODO:
//  - Implement fusion logic for fused target/pose selection (FUSED_TARGET_INFO).
//  - Consider adding internal synchronization or atomics if concurrent access is required.
//  - Improve selection heuristics and add unit tests for ProcessOutputOption and GetBestPose.

// C++ Includes
#include <algorithm> // <<-- added for std::max_element / std::distance
#include <iterator>	 // added for std::make_move_iterator
#include <memory>
#include <string>
#include <vector>

// FRC
#include "frc/RobotController.h"
#include "frc/Timer.h"
#include "frc/geometry/Pose2d.h"

// Team 302 includes
#include "chassis/ChassisConfigMgr.h"
#include "utils/DragonField.h"
#include "utils/FMSData.h"
#include "utils/logging/debug/Logger.h"
#include "vision/DragonLimelight.h"
#include "vision/DragonQuest.h"
#include "vision/DragonVision.h"
#include "vision/DragonVisionPoseEstimatorStruct.h"
#include "vision/VisionPose.h"
#include "vision/definitions/CameraConfigMgr.h"

// Third Party Includes
#include "Limelight/LimelightHelpers.h"

namespace
{
	std::vector<int> ProcessOutputOption(VisionTargetOption option, std::vector<std::unique_ptr<DragonVisionStruct>> &targets);

	std::optional<VisionPose> GetBestPose(const std::vector<VisionPose> &poses);
} // namespace

DragonVision *DragonVision::m_dragonVision = nullptr;

/// @brief Get the singleton instance of DragonVision.
/// @note Not thread-safe. Callers must synchronize if called from multiple threads
///       during initialization. The returned pointer has program lifetime; no deletion
///       is performed by the class.
/// @return Pointer to the DragonVision singleton instance.
DragonVision *DragonVision::GetDragonVision()
{
	if (DragonVision::m_dragonVision == nullptr)
	{
		DragonVision::m_dragonVision = new DragonVision();
	}
	return DragonVision::m_dragonVision;
}

/// @brief Check health for all limelights that match a usage.
/// @param usage The camera usage category to check (e.g., APRIL_TAGS).
/// @return true if all matching cameras that are considered return running; false otherwise.
/// @note If no cameras match the usage, the function will return false.
/// @thread-safety Not synchronized; callers should ensure safe concurrent access.
bool DragonVision::HealthCheck(DRAGON_LIMELIGHT_CAMERA_USAGE usage)
{
	bool isHealthy = false;
	auto limelights = GetLimelights(usage);
	for (auto limelight : limelights)
	{
		isHealthy = limelight->IsLimelightRunning();
		if (!isHealthy)
		{
			return isHealthy;
		}
	}
	return isHealthy;
}

/// @brief Check health for a single limelight identified by identifier.
/// @param identifier The camera identifier to check.
/// @return true if the camera exists and reports as running; false otherwise.
bool DragonVision::HealthCheck(DRAGON_LIMELIGHT_CAMERA_IDENTIFIER identifier)
{
	auto camera = GetLimelightFromIdentifier(identifier);
	if (camera != nullptr)
	{
		return camera->IsLimelightRunning();
	}
	return false;
}

frc::AprilTagFieldLayout DragonVision::m_aprilTagLayout = frc::AprilTagFieldLayout();

/// @brief Returns a cached AprilTag field layout, loading the 2025 field on first access.
/// @return An instance of frc::AprilTagFieldLayout describing the field tag positions.
/// @note Calling repeatedly returns the cached layout; expensive load is only done once.
frc::AprilTagFieldLayout DragonVision::GetAprilTagLayout()
{
	if (DragonVision::m_aprilTagLayout != frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025ReefscapeWelded))
	{
		DragonVision::m_aprilTagLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025ReefscapeWelded);
	}
	return DragonVision::m_aprilTagLayout;
}

/// @brief Constructor. Initializes camera configuration using the robot's team number.
/// @note Lightweight; may perform camera config initialization that should be called early
///       in robot startup.
DragonVision::DragonVision()
{
	int32_t teamNumber = frc::RobotController::GetTeamNumber();
	CameraConfigMgr::GetInstance()->InitCameras(static_cast<RobotIdentifier>(teamNumber));
}

/// @brief Add a Limelight instance to the manager.
/// @param camera Pointer to the DragonLimelight to add.
/// @param usage The camera usage category for this camera.
/// @note Ownership is not transferred here; this class stores the raw pointer.
void DragonVision::AddLimelight(DragonLimelight *camera, DRAGON_LIMELIGHT_CAMERA_USAGE usage)
{
	m_dragonLimelightMap.insert(std::pair<DRAGON_LIMELIGHT_CAMERA_USAGE, DragonLimelight *>(usage, camera));
}

/// @brief Register the DragonQuest instance with the manager.
/// @param quest Pointer to the DragonQuest instance.
/// @note Ownership not transferred; this class keeps the raw pointer.
void DragonVision::AddQuest(DragonQuest *quest)
{
	m_dragonQuest = quest;
}

/// @brief Aggregate AprilTag targets from all relevant limelights and select according to option.
/// @param option Selection option (e.g., closest valid target).
/// @param validAprilTagIDs List of AprilTag IDs considered valid for selection/filtering.
/// @return Vector of unique_ptr<DragonVisionStruct> for the selected targets. Ownership of
///         returned pointers is transferred to the caller.
/// @note Internally moves target objects from per-camera containers into the returned vector.
std::vector<std::unique_ptr<DragonVisionStruct>> DragonVision::GetAprilTagVisionTargetInfo(VisionTargetOption option,
																						   const std::vector<FieldAprilTagIDs> &validAprilTagIDs) const

{

	std::vector<std::unique_ptr<DragonVisionStruct>> alltargets;
	auto limelights = GetLimelights(DRAGON_LIMELIGHT_CAMERA_USAGE::APRIL_TAGS);
	for (auto limelight : limelights)
	{
		auto camTargets = limelight->GetAprilTagVisionTargetInfo(validAprilTagIDs);
		// move entries from camTargets into targets
		alltargets.insert(
			alltargets.end(),
			std::make_move_iterator(camTargets.begin()),
			std::make_move_iterator(camTargets.end()));
	}

	int slot = 0;
	auto indecies = ProcessOutputOption(option, alltargets);
	std::vector<std::unique_ptr<DragonVisionStruct>> targets;
	for (auto &index : indecies)
	{
		targets.emplace_back(std::move(alltargets[index]));
		slot++;
	}
	return targets;
}

/// @brief Aggregate object-detection targets from all relevant limelights and select according to option.
/// @param option Selection option (e.g., closest valid target).
/// @param validClasses List of detection class IDs considered valid for selection/filtering.
/// @return Vector of unique_ptr<DragonVisionStruct> for the selected targets. Ownership of
///         returned pointers is transferred to the caller.
/// @note Internally moves target objects from per-camera containers into the returned vector.
std::vector<std::unique_ptr<DragonVisionStruct>> DragonVision::GetObjectDetectionTargetInfo(VisionTargetOption option,
																							const std::vector<int> &validClasses) const
{
	std::vector<std::unique_ptr<DragonVisionStruct>> alltargets;
	auto limelights = GetLimelights(DRAGON_LIMELIGHT_CAMERA_USAGE::OBJECT_DETECTION_ALGAE);
	for (auto limelight : limelights)
	{
		auto camTargets = limelight->GetObjectDetectionTargetInfo(validClasses);
		// move entries from camTargets into targets
		alltargets.insert(
			alltargets.end(),
			std::make_move_iterator(camTargets.begin()),
			std::make_move_iterator(camTargets.end()));
	}

	int slot = 0;
	auto indecies = ProcessOutputOption(option, alltargets);
	std::vector<std::unique_ptr<DragonVisionStruct>> targets;
	for (auto &index : indecies)
	{
		targets.emplace_back(std::move(alltargets[index]));
		slot++;
	}
	return targets;
}

/// @brief Query all registered limelights for MegaTag1-based robot poses and choose the best.
/// @return Optional VisionPose; std::nullopt if no valid poses were returned by cameras.
/// @note Uses GetBestPose to pick the most reliable pose among candidates.
std::optional<VisionPose> DragonVision::GetRobotPositionMegaTag1()
{
	std::vector<VisionPose> poses;
	auto limelights = GetLimelights(DRAGON_LIMELIGHT_CAMERA_USAGE::APRIL_TAGS);
	for (auto limelight : limelights)
	{
		auto pose = limelight->GetMegaTag1Pose();
		if (pose.has_value())
		{
			poses.emplace_back(pose.value());
		}
	}
	return GetBestPose(poses);
}

/// @brief Query all registered limelights for MegaTag2-based robot poses and choose the best.
/// @return Optional VisionPose; std::nullopt if no valid poses were returned by cameras.
/// @note Uses GetBestPose to pick the most reliable pose among candidates.
std::optional<VisionPose> DragonVision::GetRobotPositionMegaTag2()
{
	std::vector<VisionPose> poses;
	auto limelights = GetLimelights(DRAGON_LIMELIGHT_CAMERA_USAGE::APRIL_TAGS);
	for (auto limelight : limelights)
	{
		auto pose = limelight->GetMegaTag2Pose();
		if (pose.has_value())
		{
			poses.emplace_back(pose.value());
		}
	}
	return GetBestPose(poses);
}

/// @brief Get robot pose estimate derived from Quest detections.
/// @return DragonVisionPoseEstimatorStruct - confidence level indicates the usefulness of the pose.
DragonVisionPoseEstimatorStruct DragonVision::GetRobotPositionQuest()
{
	auto quest = DragonVision::GetDragonVision()->GetQuest();
	if (quest != nullptr && quest->HealthCheck())
	{
		return quest->GetPoseEstimate();
	}
	return {};
}

/// @brief Set a robot pose for all vision subsystems that consume robot pose information.
/// @param pose The new robot pose (frc::Pose2d) to distribute.
/// @note Updates all running limelights and the DragonQuest instance (if present).
void DragonVision::SetRobotPose(const frc::Pose2d &pose)
{
	auto limelights = GetLimelights(DRAGON_LIMELIGHT_CAMERA_USAGE::ALGAE_AND_APRIL_TAGS);
	for (auto limelight : limelights)
	{
		limelight->SetRobotPose(pose);
	}

	auto quest = GetQuest();
	if (quest != nullptr)
	{
		quest->SetRobotPose(pose);
	}
}

/// @brief Return limelights that match the provided usage/category and are running.
/// @param usage The usage/category to filter by (ALGAE_AND_APRIL_TAGS, APRIL_TAGS, etc.).
/// @return Vector of pointers to running DragonLimelight instances that match the criteria.
/// @note Only returns cameras that report IsLimelightRunning() == true.
std::vector<DragonLimelight *> DragonVision::GetLimelights(DRAGON_LIMELIGHT_CAMERA_USAGE usage) const
{
	std::vector<DragonLimelight *> validLimelights;
	for (auto it = m_dragonLimelightMap.begin(); it != m_dragonLimelightMap.end(); ++it)
	{
		bool addCam = false;
		auto limelight = (*it).second;
		if (usage == DRAGON_LIMELIGHT_CAMERA_USAGE::ALGAE_AND_APRIL_TAGS)
		{
			if (limelight->IsLimelightRunning())
			{
				validLimelights.emplace_back(limelight);
			}
		}
		else
		{

			addCam = (*it).first == usage;
			if (!addCam)
			{
				if ((*it).first == DRAGON_LIMELIGHT_CAMERA_USAGE::ALGAE_AND_APRIL_TAGS)
				{
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
			if (limelight->IsLimelightRunning())
			{
				validLimelights.emplace_back(limelight);
			}
		}
	}
	return validLimelights;
}

/// @brief Lookup a limelight instance by its camera identifier.
/// @param identifier The identifier to search for.
/// @return Pointer to the DragonLimelight if found and running; nullptr otherwise.
DragonLimelight *DragonVision::GetLimelightFromIdentifier(DRAGON_LIMELIGHT_CAMERA_IDENTIFIER identifier) const
{
	auto limelights = GetLimelights(DRAGON_LIMELIGHT_CAMERA_USAGE::ALGAE_AND_APRIL_TAGS);
	for (auto limelight : limelights)
	{
		if (limelight->GetCameraIdentifier() == identifier)
		{
			return limelight;
		}
		return nullptr;
	}
	return nullptr;
}

/// @brief Set the active pipeline for all limelights that match the usage.
/// @param usage The camera usage/category whose cameras should be switched.
/// @param pipeline The pipeline enum value to set on matching cameras.
/// @note Only running cameras will be updated.
void DragonVision::SetPipeline(DRAGON_LIMELIGHT_CAMERA_USAGE usage, DRAGON_LIMELIGHT_PIPELINE pipeline)
{
	auto limelights = GetLimelights(usage);
	for (auto limelight : limelights)
	{
		limelight->SetPipeline(pipeline);
	}
}

namespace
{
	/// @brief Choose indices from a list of targets according to a selection option.
	/// @param option Selection option (e.g., CLOSEST_VALID_TARGET).
	/// @param targets Vector of unique_ptr<DragonVisionStruct> from which to select.
	/// @return Vector of indices into the provided targets vector that were selected.
	/// @note The function does not modify the target contents; callers may move entries
	///       after receiving indices. Currently only CLOSEST_VALID_TARGET is implemented.
	std::vector<int> ProcessOutputOption(VisionTargetOption option, std::vector<std::unique_ptr<DragonVisionStruct>> &targets)
	{
		std::vector<int> selectedIndices;
		if (targets.empty())
		{
			return selectedIndices;
		}

		switch (option)
		{
		case VisionTargetOption::CLOSEST_VALID_TARGET:
		{
			// find index of target with largest targetAreaPercent (closest)
			auto it = std::max_element(
				targets.begin(),
				targets.end(),
				[](const std::unique_ptr<DragonVisionStruct> &a, const std::unique_ptr<DragonVisionStruct> &b)
				{
					return a->targetAreaPercent < b->targetAreaPercent;
				});
			int index = static_cast<int>(std::distance(targets.begin(), it));
			selectedIndices.emplace_back(index);
			break;
		}
		// TODO: add fused option here
		default:
			break;
		}
		return selectedIndices;
	}

	/// @brief From a list of candidate VisionPose objects, return the one with the best (lowest) stddevs.
	/// @param poses The candidate poses to evaluate.
	/// @return Optional VisionPose chosen as best; std::nullopt if empty input.
	/// @note Compares visionMeasurementStdDevs [x, y, rot] and returns the pose with smallest combined deviations.
	std::optional<VisionPose> GetBestPose(const std::vector<VisionPose> &poses)
	{
		if (poses.empty())
		{
			return std::nullopt;
		}

		if (poses.size() == 1)
		{
			return poses[0];
		}

		auto bestfit = 0;
		auto stddevX = poses[0].visionMeasurementStdDevs[0];
		auto stddevY = poses[0].visionMeasurementStdDevs[1];
		auto stddevRot = poses[0].visionMeasurementStdDevs[2];

		auto slot = 0;
		for (auto thispose : poses)
		{
			auto thisStddevX = thispose.visionMeasurementStdDevs[0];
			auto thisStddevY = thispose.visionMeasurementStdDevs[1];
			auto thisStddevRot = thispose.visionMeasurementStdDevs[2];
			if (thisStddevX < stddevX && thisStddevY < stddevY && thisStddevRot < stddevRot)
			{
				bestfit = slot;
				stddevX = thisStddevX;
				stddevY = thisStddevY;
				stddevRot = thisStddevRot;
			}
			slot++;
		}
		return poses[bestfit];
	}
}
