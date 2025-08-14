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
#include "chassis/pose/DragonSwervePoseEstimator.h"
#include "chassis/ChassisConfigMgr.h"
#include "state/RobotState.h"
#include "state/RobotStateChanges.h"
#include "vision/DragonVision.h"

DragonSwervePoseEstimator *DragonSwervePoseEstimator::m_instance = nullptr;

DragonSwervePoseEstimator::DragonSwervePoseEstimator()
{
    m_chassis = ChassisConfigMgr::GetInstance()->GetSwerveChassis();
    m_visionPoseEstimators.clear();
}

DragonSwervePoseEstimator *DragonSwervePoseEstimator::GetInstance()
{
    if (DragonSwervePoseEstimator::m_instance == nullptr)
    {
        DragonSwervePoseEstimator::m_instance = new DragonSwervePoseEstimator();
    }
    return DragonSwervePoseEstimator::m_instance;
}

void DragonSwervePoseEstimator::RegisterVisionPoseEstimator(DragonVisionPoseEstimator *poseEstimator)
{
    if (poseEstimator != nullptr)
    {
        m_visionPoseEstimators.push_back(poseEstimator);
    }
}

// This is the new main loop method
void DragonSwervePoseEstimator::Update()
{
    if (m_chassis != nullptr)
    {
        AddVisionMeasurements();
    }
}

void DragonSwervePoseEstimator::AddVisionMeasurements()
{
    // This logic is mostly the same, but it calls the chassis method.
    for (auto estimator : m_visionPoseEstimators)
    {
        // "Pull" the data from the vision system
        auto poseInfo = estimator->GetPoseEstimate();
        if (poseInfo.m_confidenceLevel != DragonVisionPoseEstimatorStruct::ConfidenceLevel::NONE)
        {
            // "Push" the data to the chassis's internal estimator
            m_chassis->AddVisionMeasurement(poseInfo.m_visionPose,
                                            poseInfo.m_timeStamp,
                                            poseInfo.m_stds);
        }
    }
}

frc::Pose2d DragonSwervePoseEstimator::GetPose() const
{
    return (m_chassis != nullptr) ? m_chassis->GetPose() : frc::Pose2d{};
}

void DragonSwervePoseEstimator::ResetPosition(const frc::Pose2d &pose)
{
    if (m_chassis != nullptr)
    {
        m_chassis->ResetPose(pose);
    }
}

void DragonSwervePoseEstimator::CalculateInitialPose()
{
    auto vision = DragonVision::GetDragonVision();
    if (vision != nullptr)
    {
        // try making sure MegaTag1 has a good position before resetting pose to avoid screwing up MegaTag2 && Quest
        auto megaTag1Position = vision->GetRobotPosition(); // Megatag1
        if (megaTag1Position.has_value())
        {
            auto visionpose = vision->CalcVisionPose();
            if (visionpose != std::nullopt) // may want to use reset Position instead of reset pose here?
            {
                ResetPosition(visionpose.value());
            }
        }
    }
}