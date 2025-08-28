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
#include "vision/DragonVision.h"
#include "vision/DragonLimelight.h"
#include "utils/FMSData.h"
#include "vision/DragonVisionStructLogger.h"
#include "utils/logging/debug/Logger.h"
#include "utils/DragonField.h"

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
		isHealthy = cam->HealthCheck();
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
		return camera->HealthCheck();
	}
	return false;
}

std::optional<frc::Pose2d> DragonVision::CalcVisionPose()
{
	std::optional<VisionPose> megaTag1Position = GetRobotPosition(); // Megatag1
	if (megaTag1Position.has_value())
	{
		auto megaTag2Position = GetRobotPositionMegaTag2(megaTag1Position.value().estimatedPose.ToPose2d().Rotation().Degrees(),
														 units::angular_velocity::degrees_per_second_t(0.0),
														 units::angle::degree_t(0.0),
														 units::angular_velocity::degrees_per_second_t(0.0),
														 units::angle::degree_t(0.0),
														 units::angular_velocity::degrees_per_second_t(0.0));
		if (megaTag2Position.has_value())
		{

			return megaTag2Position.value().estimatedPose.ToPose2d();
		}
		return megaTag1Position.value().estimatedPose.ToPose2d();
	}

	return std::nullopt;
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
	m_poseEstimators.push_back(camera);
}

std::optional<VisionData> DragonVision::GetVisionData(VISION_ELEMENT element)
{
	if (element == VISION_ELEMENT::ALGAE)
	{
		return GetVisionDataFromAlgae(element);
	}
	else if (element == VISION_ELEMENT::NEAREST_APRILTAG) // nearest april tag
	{
		return GetVisionDataToNearestTag();
	}
	else if (element == VISION_ELEMENT::REEF)
	{
		return GetVisionDataToNearestFieldElementAprilTag(element);
	}
	else if (element == VISION_ELEMENT::BARGE)
	{
		return GetVisionDataToNearestFieldElementAprilTag(element);
	}
	else
	{
		return GetVisionDataFromElement(element);
	}
	// if we don't see any vision targets, return null optional
	return std::nullopt;
}

std::optional<VisionData> DragonVision::GetVisionDataToNearestFieldElementAprilTag(VISION_ELEMENT element)
{
	auto cameras = GetCameras(DRAGON_LIMELIGHT_CAMERA_USAGE::APRIL_TAGS);
	for (auto cam : cameras)
	{
		std::optional<VisionData> limelightData = cam->GetDataToNearestAprilTag();
		if (limelightData.has_value())
		{
			// get alliance color from FMSData
			frc::DriverStation::Alliance allianceColor = FMSData::GetAllianceColor();

			// initialize tags to check to null pointer
			std::vector<int> tagIdsToCheck = {};
			switch (element)
			{
			case VISION_ELEMENT::REEF:
				tagIdsToCheck = GetReefTags(allianceColor);
				break;

			case VISION_ELEMENT::PROCESSOR:
				tagIdsToCheck = GetProcessorTags(allianceColor);
				break;

			case VISION_ELEMENT::CORAL_STATION:
				tagIdsToCheck = GetCoralStationsTags(allianceColor);
				break;

			case VISION_ELEMENT::BARGE:
				tagIdsToCheck = GetBargeTags(allianceColor);
				break;

			default:
				return std::nullopt;
				break;
			}

			if (std::find(tagIdsToCheck.begin(), tagIdsToCheck.end(), limelightData.value().tagId) != tagIdsToCheck.end())
			{
				return limelightData;
			}
		}
	}

	// tag doesnt matter or no tag
	return std::nullopt;
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

std::optional<VisionData> DragonVision::GetVisionDataToNearestTag()
{
	std::optional<VisionData> limelightData = std::nullopt;

	units::length::inch_t closest = units::length::inch_t(-1.0);
	std::vector<VisionData> visData;
	auto cameras = GetCameras(DRAGON_LIMELIGHT_CAMERA_USAGE::APRIL_TAGS);
	for (auto cam : cameras)
	{
		auto data = cam->GetDataToNearestAprilTag();
		if (data.has_value())
		{
			units::length::inch_t dist = data.value().translationToTarget.X();
			if (closest.value() < 0.0 || closest.value() > dist.value())
			{
				closest = dist;
				limelightData = data;
			}
		}
	}

	return limelightData;
}

std::optional<VisionData> DragonVision::GetVisionDataFromAlgae(VISION_ELEMENT element)
{
	auto cameras = GetCameras(DRAGON_LIMELIGHT_CAMERA_USAGE::ALGAE_AND_APRIL_TAGS);
	return GetRawVisionDataFromObject(cameras, DRAGON_LIMELIGHT_PIPELINE::MACHINE_LEARNING_PL);
}

std::optional<VisionData> DragonVision::GetRawVisionDataFromObject(std::vector<DragonLimelight *> cameras, DRAGON_LIMELIGHT_PIPELINE pipeline)
{
	for (auto cam : cameras)
	{
		if (cam->GetPipeline() == DRAGON_LIMELIGHT_PIPELINE::MACHINE_LEARNING_PL)
		{
			if (cam->HasTarget())
			{
				// create translation using 3 estimated distances
				if (cam->EstimateTargetXDistance_RelToRobotCoords().has_value() ||
					cam->EstimateTargetZDistance_RelToRobotCoords().has_value() ||
					cam->EstimateTargetYDistance_RelToRobotCoords().has_value())
				{
					frc::Translation3d translationToTarget = frc::Translation3d(cam->EstimateTargetXDistance_RelToRobotCoords().value(),
																				cam->EstimateTargetYDistance_RelToRobotCoords().value(),
																				cam->EstimateTargetZDistance_RelToRobotCoords().value());
					frc::Rotation3d rotationToTarget = frc::Rotation3d();

					// create rotation3d with pitch and yaw (don't have access to roll)
					rotationToTarget = frc::Rotation3d(units::angle::degree_t(0.0),
													   cam->GetTargetPitchRobotFrame().value(),
													   cam->GetTargetYawRobotFrame().value());

					// return VisionData with new translation and rotation
					return VisionData{frc::Transform3d(translationToTarget, rotationToTarget), translationToTarget, rotationToTarget, -1};
				}
			}
		}
	}
	// if we don't have a selected cam
	return std::nullopt;
}

std::optional<VisionData> DragonVision::GetVisionDataFromElement(VISION_ELEMENT element)
{
	frc::DriverStation::Alliance allianceColor = FMSData::GetAllianceColor();

	// initialize selected field element to empty Pose3d
	frc::Pose3d fieldElementPose = frc::Pose3d{};
	int idToSearch = -1;
	switch (element)
	{
		// TODO: JW need to handle multiple tags
	case VISION_ELEMENT::REEF:
		fieldElementPose = allianceColor == frc::DriverStation::Alliance::kRed ? frc::Pose3d{FieldConstants::GetInstance()->GetFieldElementPose(FieldConstants::FIELD_ELEMENT::RED_REEF_CENTER)} /*load red reef*/ : frc::Pose3d{FieldConstants::GetInstance()->GetFieldElementPose(FieldConstants::FIELD_ELEMENT::BLUE_REEF_CENTER)}; /*load blue reef*/
		idToSearch = allianceColor == frc::DriverStation::Alliance::kRed ? 4 : 7;																																																													   // should be red 6-11, blue 17-22
		break;
	case VISION_ELEMENT::PROCESSOR:
		fieldElementPose = allianceColor == frc::DriverStation::Alliance::kRed ? frc::Pose3d{FieldConstants::GetInstance()->GetFieldElementPose(FieldConstants::FIELD_ELEMENT::RED_PROCESSOR)} /*load red processor*/ : frc::Pose3d{FieldConstants::GetInstance()->GetFieldElementPose(FieldConstants::FIELD_ELEMENT::BLUE_PROCESSOR)}; /*load blue processor*/
		idToSearch = allianceColor == frc::DriverStation::Alliance::kRed ? 5 : 6;
		break;
	case VISION_ELEMENT::CORAL_STATION:
		fieldElementPose = allianceColor == frc::DriverStation::Alliance::kRed ? frc::Pose3d{FieldConstants::GetInstance()->GetFieldElementPose(FieldConstants::FIELD_ELEMENT::RED_CORAL_STATION_LEFT_SIDEWALL)} /*load red coral station*/ : frc::Pose3d{FieldConstants::GetInstance()->GetFieldElementPose(FieldConstants::FIELD_ELEMENT::BLUE_CORAL_STATION_LEFT_SIDEWALL)}; /*load blue coral station*/
		idToSearch = allianceColor == frc::DriverStation::Alliance::kRed ? 5 : 6;																																																																								// should be red 1 or 2, blue 12 or 13
		break;
	case VISION_ELEMENT::BARGE:
		fieldElementPose = allianceColor == frc::DriverStation::Alliance::kRed ? frc::Pose3d{FieldConstants::GetInstance()->GetFieldElementPose(FieldConstants::FIELD_ELEMENT::RED_BARGE_FRONT)} /*load red barge*/ : frc::Pose3d{FieldConstants::GetInstance()->GetFieldElementPose(FieldConstants::FIELD_ELEMENT::BLUE_BARGE_FRONT)}; /*load blue barge*/
		idToSearch = allianceColor == frc::DriverStation::Alliance::kRed ? 5 : 6;																																																														// should be red 5 or 15, blue 4 or 14
		break;
	default:
		return std::nullopt;
		break;
	}

	std::optional<VisionData> singleTagEstimate = SingleTagToElement(fieldElementPose, idToSearch);
	if (singleTagEstimate)
	{
		return singleTagEstimate;
	}

	return std::nullopt;
}

std::optional<VisionData> DragonVision::SingleTagToElement(frc::Pose3d elementPose, int idToSearch)
{
	auto cameras = GetCameras(DRAGON_LIMELIGHT_CAMERA_USAGE::APRIL_TAGS);
	for (auto cam : cameras)
	{
		// get the optional of the translation and rotation to the apriltag
		auto aprilTagData = cam->GetDataToSpecifiedTag(idToSearch);
		if (aprilTagData.has_value())
		{
			return aprilTagData;
		}
	}

	return std::nullopt;
}

std::optional<VisionPose> DragonVision::GetRobotPosition()
{
	auto cameras = GetCameras(DRAGON_LIMELIGHT_CAMERA_USAGE::APRIL_TAGS);
	for (auto cam : cameras)
	{
		auto pose = cam->EstimatePoseOdometryLimelight(false); // false megatag1
		if (pose.has_value())								   // if we have a valid pose, return it
		{
			return pose;
		}
	}

	// if we aren't able to calculate our pose from vision, return a null optional
	return std::nullopt;
}

std::optional<VisionPose> DragonVision::GetRobotPositionMegaTag2(units::angle::degree_t yaw,
																 units::angular_velocity::degrees_per_second_t yawRate,
																 units::angle::degree_t pitch,
																 units::angular_velocity::degrees_per_second_t pitchRate,
																 units::angle::degree_t roll,
																 units::angular_velocity::degrees_per_second_t rollRate)
{
	auto cameras = GetCameras(DRAGON_LIMELIGHT_CAMERA_USAGE::APRIL_TAGS);
	for (auto cam : cameras)
	{
		LimelightHelpers::SetRobotOrientation(cam->GetCameraName(),
											  yaw.value(),
											  yawRate.value(),
											  pitch.value(),
											  pitchRate.value(),
											  roll.value(),
											  rollRate.value());
		auto estPose = cam->EstimatePoseOdometryLimelight(true); // true since megatag2
		if (estPose.has_value())
		{
			return estPose;
		}
	}

	return std::nullopt;
}

/*****************
 * testAndLogVisionData:  Test and log the vision data
 * add this line to teleopPeriodic to test and log vision data
 *
 *     DragonVision::GetDragonVision()->testAndLogVisionData();
 */
void DragonVision::testAndLogVisionData()
{
	try
	{
		std::optional<VisionPose> visionPose = GetRobotPosition();

		DragonField::GetInstance()->UpdateObjectVisionPose("VisionPose", visionPose);
	}
	catch (std::bad_optional_access &boa)
	{
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, std::string("testAndLogVisionData"), std::string("bad_optional_access"), boa.what());
	}
	catch (std::exception &e)
	{
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, std::string("testAndLogVisionData"), std::string("exception"), e.what());
	}
}

// Limelight raw data functions

// TODO:  these need to be smarter to deal with multiple cameras with the same usage
std::optional<double> DragonVision::GetTargetArea(DRAGON_LIMELIGHT_CAMERA_USAGE usage)
{
	auto cameras = GetCameras(usage);
	std::optional<double> maxArea = std::nullopt;
	for (auto cam : cameras)
	{
		auto thisArea = cam->GetTargetArea();
		if (thisArea.has_value())
		{
			if (!maxArea.has_value() || thisArea.value() > maxArea.value())
			{
				maxArea = thisArea;
			}
		}
	}
	return maxArea;
}
units::angle::degree_t DragonVision::GetTy(DRAGON_LIMELIGHT_CAMERA_USAGE usage)
{
	auto cameras = GetCameras(usage);
	units::angle::degree_t minTy = units::angle::degree_t(720); // arbitrary large value
	for (auto cam : cameras)
	{
		auto thisTy = cam->GetTy();
		if (thisTy < minTy)
		{
			minTy = thisTy;
		}
	}
	return minTy;
}

units::angle::degree_t DragonVision::GetTx(DRAGON_LIMELIGHT_CAMERA_USAGE usage)
{
	auto cameras = GetCameras(usage);
	units::angle::degree_t minTx = units::angle::degree_t(720); // arbitrary large value
	for (auto cam : cameras)
	{
		auto thisTx = cam->GetTx();
		if (thisTx < minTx)
		{
			minTx = thisTx;
		}
	}
	return minTx;
}
std::optional<units::angle::degree_t> DragonVision::GetTargetYaw(DRAGON_LIMELIGHT_CAMERA_USAGE usage)
{
	auto cameras = GetCameras(usage);
	std::optional<units::angle::degree_t> minYaw = std::nullopt;
	for (auto cam : cameras)
	{
		auto thisYaw = cam->GetTargetYaw();
		if (thisYaw.has_value())
		{
			if (!minYaw.has_value() || thisYaw.value() < minYaw.value())
			{
				minYaw = thisYaw;
			}
		}
	}
	return minYaw;
}

std::optional<units::angle::degree_t> DragonVision::GetTargetSkew(DRAGON_LIMELIGHT_CAMERA_USAGE usage)
{
	auto cameras = GetCameras(usage);
	std::optional<units::angle::degree_t> minSkew = std::nullopt;
	for (auto cam : cameras)
	{
		auto thisSkew = cam->GetTargetSkew();
		if (thisSkew.has_value())
		{
			if (!minSkew.has_value() || thisSkew.value() < minSkew.value())
			{
				minSkew = thisSkew;
			}
		}
	}
	return minSkew;
}

std::optional<units::angle::degree_t> DragonVision::GetTargetPitch(DRAGON_LIMELIGHT_CAMERA_USAGE usage)
{
	auto cameras = GetCameras(usage);
	std::optional<units::angle::degree_t> minPitch = std::nullopt;
	for (auto cam : cameras)
	{
		auto thisPitch = cam->GetTargetPitch();
		if (thisPitch.has_value())
		{
			if (!minPitch.has_value() || thisPitch.value() < minPitch.value())
			{
				minPitch = thisPitch;
			}
		}
	}
	return minPitch;
}

std::optional<int> DragonVision::GetAprilTagID(DRAGON_LIMELIGHT_CAMERA_USAGE usage)
{
	auto cameras = GetCameras(usage);
	std::optional<int> targetAprilTag = std::nullopt;
	std::optional<double> minArea = std::nullopt;
	for (auto cam : cameras)
	{
		auto thisTag = cam->GetAprilTagID();
		if (thisTag.has_value())
		{
			auto thisArea = cam->GetTargetArea();
			if (!minArea.has_value() || thisArea.value() < minArea.value())
			{
				minArea = thisArea;
				targetAprilTag = thisTag;
			}
		}
	}
	return targetAprilTag;
}

bool DragonVision::HasTarget(DRAGON_LIMELIGHT_CAMERA_USAGE usage)
{
	auto cameras = GetCameras(usage);
	for (auto cam : cameras)
	{
		auto hasTarget = cam->HasTarget();
		if (hasTarget)
		{
			return hasTarget;
		}
	}
	return false;
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
			if (cam->HealthCheck())
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
					auto pipe = cam->GetPipeline();
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
			if (cam->HealthCheck())
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

std::optional<frc::Pose3d> DragonVision::GetAprilTagPose(FieldConstants::AprilTagIDs tagId) const
{
	auto cameras = GetCameras(DRAGON_LIMELIGHT_CAMERA_USAGE::APRIL_TAGS);
	for (auto cam : cameras)
	{
		Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("DragonTargetFinder"), std::string("DragonVision - cam"), cam->GetCameraName());
		Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("DragonTargetFinder"), std::string("DragonVision - tagID"), static_cast<int>(tagId));

		auto visdata = cam->GetDataToSpecifiedTag(static_cast<int>(tagId));
		Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("DragonTargetFinder"), std::string("DragonVision - visData.has_value()"), visdata.has_value() ? "true" : "false");

		if (visdata.has_value())
		{
			auto currentPose{frc::Pose3d(ChassisConfigMgr::GetInstance()->GetSwerveChassis()->GetPose())};

			auto trans3d = visdata.value().transformToTarget;
			auto targetPose = currentPose + trans3d;
			units::angle::degree_t robotRelativeAngle = visdata.value().rotationToTarget.Z(); // value is robot to target

			units::angle::degree_t fieldRelativeAngle = currentPose.Rotation().Angle() + robotRelativeAngle;
			auto pose = frc::Pose2d(targetPose.X(), targetPose.Y(), fieldRelativeAngle);
			return frc::Pose3d(pose);
		}
	}

	return std::nullopt;
}

void DragonVision::SetPipeline(DRAGON_LIMELIGHT_CAMERA_USAGE position, DRAGON_LIMELIGHT_PIPELINE pipeline)
{
	auto cameras = GetCameras(position);
	for (auto cam : cameras)
	{
		if (cam->GetPipeline() != pipeline)
		{
			cam->SetPipeline(pipeline);
			cam->PeriodicCacheData();
		}
	}
}