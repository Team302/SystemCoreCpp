#include <gtest/gtest.h> //nevermind that intellisense doesn't seem to find this file
#include "fielddata/FieldConstants.h"

class FieldConstantsTest : public ::testing::Test
{
protected:
};

TEST_F(FieldConstantsTest, GetFieldElementTestReefCenter)
{
    auto fieldConstants = FieldConstants::GetInstance();

    frc::Pose3d result = fieldConstants->GetFieldElementPose(FieldConstants::FIELD_ELEMENT::RED_REEF_CENTER);

    EXPECT_EQ(result.X().to<double>(), 13.058901999999998);
    EXPECT_EQ(result.Y().to<double>(), 4.0259);
    EXPECT_EQ(result.Z().to<double>(), 0.30810199999999993);
}

TEST_F(FieldConstantsTest, GetFieldElementBlueBargeBack)
{
    auto fieldConstants = FieldConstants::GetInstance();

    frc::Pose3d result = fieldConstants->GetFieldElementPose(FieldConstants::FIELD_ELEMENT::BLUE_BARGE_BACK_CALCULATED);
    EXPECT_EQ(result.X().to<double>(), 10.089970674476614);
    EXPECT_EQ(result.Y().to<double>(), 6.1376559999999998);
    EXPECT_EQ(result.Z().to<double>(), 1.3980160000000001);
}