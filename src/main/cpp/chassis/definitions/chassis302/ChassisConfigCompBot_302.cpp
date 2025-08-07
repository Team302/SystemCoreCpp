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

#include "chassis/definitions/chassis302/ChassisConfigCompBot_302.h"
#include "chassis/SwerveModule.h"
#include "chassis/SwerveModuleConstants.h"
#include "utils/logging/debug/Logger.h"
#include "chassis/definitions/chassis302/TunerConstants302.h"

using ctre::phoenix6::configs::MountPoseConfigs;
using ctre::phoenix6::hardware::Pigeon2;
using std::string;

void ChassisConfigCompBot_302::DefinePigeon()
{
    string canbusName;
    canbusName.assign(TunerConstants302::kCANBusName);
    m_pigeon2 = new Pigeon2(TunerConstants302::kPigeonId, canbusName);
    m_pigeon2->Reset();
}

void ChassisConfigCompBot_302::DefineChassis()
{
    string networkTableName{string("swerve")};
    string canbusName;
    canbusName.assign(TunerConstants302::kCANBusName);
    units::length::inch_t wheeldiameter = TunerConstants302::kWheelRadius * 2.0;

    m_leftFrontModule = new SwerveModule(canbusName,
                                         SwerveModuleConstants::ModuleID::LEFT_FRONT,
                                         wheeldiameter,

                                         TunerConstants302::kDriveGearRatio,
                                         1.0,
                                         TunerConstants302::kSteerGearRatio,
                                         TunerConstants302::kSpeedAt12Volts,

                                         TunerConstants302::kFrontLeftDriveMotorId,
                                         TunerConstants302::kInvertLeftSide,

                                         TunerConstants302::kFrontLeftSteerMotorId,
                                         TunerConstants302::kFrontLeftSteerMotorInverted,

                                         TunerConstants302::kFrontLeftEncoderId,
                                         TunerConstants302::kFrontLeftEncoderInverted,
                                         TunerConstants302::kFrontLeftEncoderOffset,

                                         TunerConstants302::steerGains,

                                         networkTableName);

    m_leftBackModule = new SwerveModule(canbusName,
                                        SwerveModuleConstants::ModuleID::LEFT_BACK,
                                        wheeldiameter,

                                        TunerConstants302::kDriveGearRatio,
                                        1.0,
                                        TunerConstants302::kSteerGearRatio,
                                        TunerConstants302::kSpeedAt12Volts,

                                        TunerConstants302::kBackLeftDriveMotorId,
                                        TunerConstants302::kInvertLeftSide,

                                        TunerConstants302::kBackLeftSteerMotorId,
                                        TunerConstants302::kBackLeftSteerMotorInverted,

                                        TunerConstants302::kBackLeftEncoderId,
                                        TunerConstants302::kBackLeftEncoderInverted,
                                        TunerConstants302::kBackLeftEncoderOffset,

                                        TunerConstants302::steerGains,

                                        networkTableName);

    m_rightFrontModule = new SwerveModule(canbusName,
                                          SwerveModuleConstants::ModuleID::RIGHT_FRONT,
                                          wheeldiameter,

                                          TunerConstants302::kDriveGearRatio,
                                          1.0,
                                          TunerConstants302::kSteerGearRatio,
                                          TunerConstants302::kSpeedAt12Volts,

                                          TunerConstants302::kFrontRightDriveMotorId,
                                          TunerConstants302::kInvertRightSide,

                                          TunerConstants302::kFrontRightSteerMotorId,
                                          TunerConstants302::kFrontRightSteerMotorInverted,

                                          TunerConstants302::kFrontRightEncoderId,
                                          TunerConstants302::kFrontRightEncoderInverted,
                                          TunerConstants302::kFrontRightEncoderOffset,

                                          TunerConstants302::steerGains,

                                          networkTableName);

    m_rightBackModule = new SwerveModule(canbusName,
                                         SwerveModuleConstants::ModuleID::RIGHT_BACK,
                                         wheeldiameter,

                                         TunerConstants302::kDriveGearRatio,
                                         1.0,
                                         TunerConstants302::kSteerGearRatio,
                                         TunerConstants302::kSpeedAt12Volts,

                                         TunerConstants302::kBackRightDriveMotorId,
                                         TunerConstants302::kInvertRightSide,

                                         TunerConstants302::kBackRightSteerMotorId,
                                         TunerConstants302::kBackRightSteerMotorInverted,

                                         TunerConstants302::kBackRightEncoderId,
                                         TunerConstants302::kBackRightEncoderInverted,
                                         TunerConstants302::kBackRightEncoderOffset,

                                         TunerConstants302::steerGains,

                                         networkTableName);

    units::length::inch_t wheelbase = TunerConstants302::kFrontLeftXPos -
                                      TunerConstants302::kBackLeftXPos;
    units::length::inch_t trackwidth = TunerConstants302::kFrontLeftYPos -
                                       TunerConstants302::kFrontRightYPos;

    m_chassis = new SwerveChassis(m_leftFrontModule,
                                  m_rightFrontModule,
                                  m_leftBackModule,
                                  m_rightBackModule,
                                  m_pigeon2,
                                  networkTableName,
                                  wheelbase,
                                  trackwidth,
                                  wheeldiameter,
                                  TunerConstants302::kSpeedAt12Volts);
}
