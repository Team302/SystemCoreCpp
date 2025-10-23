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

#pragma once
#include "vision/definitions/CameraConfig.h"

#include "units/length.h"

class CameraConfig_302 : public CameraConfig
{
public:
    CameraConfig_302() = default;
    ~CameraConfig_302() = default;

    void BuildCameraConfig() override;

    std::vector<int> GetLimelightIndexs() override { return m_limelightIndexs; }
    int GetQuestIndex() override { return m_questIndex; }

private:
    std::vector<int> m_limelightIndexs;
    int m_questIndex = -1;

    static constexpr units::length::meter_t m_ll1MountingXOffset{0.24765};  // 9.75 inches
    static constexpr units::length::meter_t m_ll1MountingYOffset{-0.08255}; // -3.25 inches
    static constexpr units::length::meter_t m_ll1MountingZOffset{0.8223};   // 32.4 inches
    static constexpr units::angle::degree_t m_ll1Pitch{-32.0};              // -32 degrees
    static constexpr units::angle::degree_t m_ll1Yaw{0};                    // 0 degrees
    static constexpr units::angle::degree_t m_ll1Roll{0};                   // 0 degrees

    static constexpr units::length::inch_t m_ll2MountingXOffset{9.663};  // 9.663 inches
    static constexpr units::length::inch_t m_ll2MountingYOffset{11.101}; // 11.101 inches Left
    static constexpr units::length::inch_t m_ll2MountingZOffset{9.639};  // 9.639 inches
    static constexpr units::angle::degree_t m_ll2Pitch{6.8};             // 6.8 degrees up
    static constexpr units::angle::degree_t m_ll2Yaw{-40};               // 40 degrees to the right
    static constexpr units::angle::degree_t m_ll2Roll{0};

    static constexpr units::length::inch_t m_questMountingXOffset{-12.3};
    static constexpr units::length::inch_t m_questMountingYOffset{-2.5};
    static constexpr units::length::inch_t m_questMountingZOffset{12.574};
    static constexpr units::angle::degree_t m_questPitch{0.0};
    static constexpr units::angle::degree_t m_questYaw{180.0};
    static constexpr units::angle::degree_t m_questRoll{0.0};
};