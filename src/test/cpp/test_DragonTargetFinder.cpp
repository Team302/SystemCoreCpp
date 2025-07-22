#include <gtest/gtest.h>
#include <frc/RobotController.h>
#include "fielddata/DragonTargetFinder.h"
#include "chassis/ChassisConfigMgr.h"
#include "vision/DragonVision.h"
#include "units/angle.h"
#include "utils/FMSData.h"
#include "frc/DriverStation.h"

class DragonTargetFinderTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Initialize necessary components
        int32_t teamNumber = frc::RobotController::GetTeamNumber();
        ChassisConfigMgr::GetInstance()->InitChassis(static_cast<RobotIdentifier>(teamNumber));
        auto chassisConfig = ChassisConfigMgr::GetInstance();
        auto chassis = chassisConfig->GetSwerveChassis();
    }

    void TearDown() override
    {
        // Clean up - there's no easy way to cleanup the singletons as they don't have deletes
    }
};

TEST_F(DragonTargetFinderTest, GetInstance)
{
    auto instance = DragonTargetFinder::GetInstance();
    ASSERT_NE(instance, nullptr);
}

TEST_F(DragonTargetFinderTest, GetPose_ReefCenter)
{
    auto instance = DragonTargetFinder::GetInstance();
    auto result = instance->GetPose(DragonTargetFinderTarget::REEF_CENTER);
    ASSERT_TRUE(result.has_value());
    auto [data, pose] = result.value();
    ASSERT_EQ(data, DragonTargetFinderData::ODOMETRY_BASED);
}

TEST_F(DragonTargetFinderTest, GetPose_ClosestLeftReefBranch)
{
    auto instance = DragonTargetFinder::GetInstance();
    auto result = instance->GetPose(DragonTargetFinderTarget::CLOSEST_LEFT_REEF_BRANCH);
    ASSERT_TRUE(result.has_value());
    auto [data, pose] = result.value();
    ASSERT_EQ(data, DragonTargetFinderData::ODOMETRY_BASED);
}

TEST_F(DragonTargetFinderTest, GetPose_ClosestRightReefBranch)
{
    auto instance = DragonTargetFinder::GetInstance();
    auto result = instance->GetPose(DragonTargetFinderTarget::CLOSEST_RIGHT_REEF_BRANCH);
    ASSERT_TRUE(result.has_value());
    auto [data, pose] = result.value();
    ASSERT_EQ(data, DragonTargetFinderData::ODOMETRY_BASED);
}

TEST_F(DragonTargetFinderTest, GetPose_ClosestReefAlgae)
{
    auto instance = DragonTargetFinder::GetInstance();
    auto result = instance->GetPose(DragonTargetFinderTarget::CLOSEST_REEF_ALGAE);
    ASSERT_TRUE(result.has_value());
    auto [data, pose] = result.value();
    ASSERT_EQ(data, DragonTargetFinderData::ODOMETRY_BASED);
}

TEST_F(DragonTargetFinderTest, GetPose_ClosestCoralStationMiddle)
{
    auto instance = DragonTargetFinder::GetInstance();
    auto result = instance->GetPose(DragonTargetFinderTarget::CLOSEST_CORAL_STATION_MIDDLE);
    ASSERT_TRUE(result.has_value());
    auto [data, pose] = result.value();
    ASSERT_EQ(data, DragonTargetFinderData::ODOMETRY_BASED);
}

TEST_F(DragonTargetFinderTest, GetPose_ClosestCoralStationSidewallSide)
{
    auto instance = DragonTargetFinder::GetInstance();
    auto result = instance->GetPose(DragonTargetFinderTarget::CLOSEST_CORAL_STATION_SIDWALL_SIDE);
    ASSERT_TRUE(result.has_value());
    auto [data, pose] = result.value();
    ASSERT_EQ(data, DragonTargetFinderData::ODOMETRY_BASED);
}

TEST_F(DragonTargetFinderTest, GetPose_ClosestCoralStationAllianceSide)
{
    auto instance = DragonTargetFinder::GetInstance();
    auto result = instance->GetPose(DragonTargetFinderTarget::CLOSEST_CORAL_STATION_ALLIANCE_SIDE);
    ASSERT_TRUE(result.has_value());
    auto [data, pose] = result.value();
    ASSERT_EQ(data, DragonTargetFinderData::ODOMETRY_BASED);
}
