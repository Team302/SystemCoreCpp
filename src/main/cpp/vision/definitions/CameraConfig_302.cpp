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

#include <string>

#include "utils/logging/debug/Logger.h"
#include "vision/definitions/CameraConfig_302.h"
#include "vision/DragonLimelight.h"
#include "vision/DragonVision.h"
#include "vision/DragonQuest.h"

void CameraConfig_302::BuildCameraConfig()
{
    DragonLimelight *front = new DragonLimelight(std::string("limelight-front"), // networkTableName
                                                 DRAGON_LIMELIGHT_CAMERA_IDENTIFIER::FRONT_CAMERA,
                                                 DRAGON_LIMELIGHT_CAMERA_TYPE::LIMELIGHT4,            // PIPELINE initialPipeline,
                                                 DRAGON_LIMELIGHT_CAMERA_USAGE::ALGAE_AND_APRIL_TAGS, // PIPELINE initialPipeline,
                                                 units::length::meter_t(m_ll1MountingXOffset),        // units::length::inch_t mountingXOffset, /// <I> x offset of cam from robot center (forward relative to robot)
                                                 units::length::meter_t(m_ll1MountingYOffset),        // units::length::inch_t mountingYOffset, /// <I> y offset of cam from robot center (left relative to robot)
                                                 units::length::meter_t(m_ll1MountingZOffset),        // units::length::inch_t mountingZOffset, /// <I> z offset of cam from robot center (up relative to robot)
                                                 units::angle::degree_t(m_ll1Pitch),                  // units::angle::degree_t pitch,          /// <I> - Pitch of Camera
                                                 units::angle::degree_t(m_ll1Yaw),                    // units::angle::degree_t yaw,            /// <I> - Yaw of Camera
                                                 units::angle::degree_t(m_ll1Roll),                   // units::angle::degree_t roll,           /// <I> - Roll of Camera
                                                 DRAGON_LIMELIGHT_PIPELINE::APRIL_TAG,                /// <I> enum for starting pipeline
                                                 DRAGON_LIMELIGHT_LED_MODE::LED_OFF,                  // DRAGON_LIMELIGHT_LED_MODE ledMode,
                                                 DRAGON_LIMELIGHT_CAM_MODE::CAM_VISION                // CAM_MODE camMode,

    ); // additional parameter
    DragonVision::GetDragonVision()->AddLimelight(front, DRAGON_LIMELIGHT_CAMERA_USAGE::ALGAE_AND_APRIL_TAGS);
    m_limelightIndexs.push_back(0);

    DragonLimelight *back = new DragonLimelight(std::string("limelight-algae"), // networkTableName
                                                DRAGON_LIMELIGHT_CAMERA_IDENTIFIER::BACK_CAMERA,
                                                DRAGON_LIMELIGHT_CAMERA_TYPE::LIMELIGHT4,     // PIPELINE initialPipeline,
                                                DRAGON_LIMELIGHT_CAMERA_USAGE::APRIL_TAGS,    // PIPELINE initialPipeline,
                                                units::length::meter_t(m_ll2MountingXOffset), // units::length::inch_t mountingXOffset, /// <I> x offset of cam from robot center (forward relative to robot)
                                                units::length::meter_t(m_ll2MountingYOffset), // units::length::inch_t mountingYOffset, /// <I> y offset of cam from robot center (left relative to robot)
                                                units::length::meter_t(m_ll2MountingZOffset), // units::length::inch_t mountingZOffset, /// <I> z offset of cam from robot center (up relative to robot)
                                                units::angle::degree_t(m_ll2Pitch),           // units::angle::degree_t pitch,          /// <I> - Pitch of Camera
                                                units::angle::degree_t(m_ll2Yaw),             // units::angle::degree_t yaw,            /// <I> - Yaw of Camera
                                                units::angle::degree_t(m_ll2Roll),            // units::angle::degree_t roll,           /// <I> - Roll of Camera
                                                DRAGON_LIMELIGHT_PIPELINE::APRIL_TAG,         /// <I> enum for starting pipeline
                                                DRAGON_LIMELIGHT_LED_MODE::LED_OFF,           // DRAGON_LIMELIGHT_LED_MODE ledMode,
                                                DRAGON_LIMELIGHT_CAM_MODE::CAM_VISION         // CAM_MODE camMode,

    ); // additional parameter
    DragonVision::GetDragonVision()->AddLimelight(back, DRAGON_LIMELIGHT_CAMERA_USAGE::APRIL_TAGS);

    new DragonQuest(units::length::inch_t(m_questMountingXOffset), // <I> x offset of Quest from robot center (forward relative to robot)
                    units::length::inch_t(m_questMountingYOffset), // <I> y offset of Quest from robot center (left relative to robot)
                    units::length::inch_t(m_questMountingZOffset), // <I> z offset of Quest from robot center (up relative to robot)
                    units::angle::degree_t(m_questPitch),          // <I> - Pitch of Quest
                    units::angle::degree_t(m_questYaw),            // <I> - Yaw of Quest
                    units::angle::degree_t(m_questRoll)            // <I> - Roll of Quest
    );
}
