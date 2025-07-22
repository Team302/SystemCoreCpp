#include <gtest/gtest.h> //nevermind that intellisense doesn't seem to find this file
#include "fielddata/FieldConstants.h"

class FieldConstantsTest : public ::testing::Test
{
protected:
};

TEST_F(FieldConstantsTest, GetFieldElementTest)
{
    auto fieldConstants = FieldConstants::GetInstance();

    frc::Pose3d result = fieldConstants->GetFieldElementPose(FieldConstants::FIELD_ELEMENT::RED_REEF_CENTER);
    EXPECT_NE(result.X().to<double>(), 5.32);
    EXPECT_NE(result.Y().to<double>(), 4.11);
    EXPECT_NE(result.Z().to<double>(), 1.3200000000000001);
    EXPECT_NE(result.Z().to<double>(), 4);
}