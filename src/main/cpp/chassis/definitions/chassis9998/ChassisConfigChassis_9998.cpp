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

#include "chassis/definitions/chassis9998/ChassisConfigChassis_9998.h"
#include "chassis/definitions/chassis9998/TunerConstants9998.h"
#include "chassis/SwerveModule.h"
#include "chassis/SwerveModuleConstants.h"
#include "utils/logging/debug/Logger.h"

using ctre::phoenix6::configs::MountPoseConfigs;
using ctre::phoenix6::hardware::Pigeon2;
using std::string;

void ChassisConfigChassis_9998::DefinePigeon()
{
    string canbusName;
    canbusName.assign(TunerConstants9998::kCANBusName);
    m_pigeon2 = new Pigeon2(TunerConstants9998::kPigeonId, canbusName);
    m_pigeon2->Reset();
}

void ChassisConfigChassis_9998::DefineChassis()
{
    string networkTableName{string("swerve")};
    string canbusName;
    canbusName.assign(TunerConstants9998::kCANBusName);
    units::length::inch_t wheeldiameter = TunerConstants9998::kWheelRadius * 2.0;

    m_leftFrontModule = new SwerveModule(canbusName,
                                         SwerveModuleConstants::ModuleID::LEFT_FRONT,
                                         wheeldiameter,

                                         TunerConstants9998::kDriveGearRatio,
                                         1.0,
                                         TunerConstants9998::kSteerGearRatio,
                                         TunerConstants9998::kSpeedAt12Volts,

                                         TunerConstants9998::kFrontLeftDriveMotorId,
                                         TunerConstants9998::kInvertLeftSide,

                                         TunerConstants9998::kFrontLeftSteerMotorId,
                                         TunerConstants9998::kFrontLeftSteerMotorInverted,

                                         TunerConstants9998::kFrontLeftEncoderId,
                                         TunerConstants9998::kFrontLeftEncoderInverted,
                                         TunerConstants9998::kFrontLeftEncoderOffset,

                                         TunerConstants9998::steerGains,

                                         networkTableName);

    m_leftBackModule = new SwerveModule(canbusName,
                                        SwerveModuleConstants::ModuleID::LEFT_BACK,
                                        wheeldiameter,

                                        TunerConstants9998::kDriveGearRatio,
                                        1.0,
                                        TunerConstants9998::kSteerGearRatio,
                                        TunerConstants9998::kSpeedAt12Volts,

                                        TunerConstants9998::kBackLeftDriveMotorId,
                                        TunerConstants9998::kInvertLeftSide,

                                        TunerConstants9998::kBackLeftSteerMotorId,
                                        TunerConstants9998::kBackLeftSteerMotorInverted,

                                        TunerConstants9998::kBackLeftEncoderId,
                                        TunerConstants9998::kBackLeftEncoderInverted,
                                        TunerConstants9998::kBackLeftEncoderOffset,

                                        TunerConstants9998::steerGains,

                                        networkTableName);

    m_rightFrontModule = new SwerveModule(canbusName,
                                          SwerveModuleConstants::ModuleID::RIGHT_FRONT,
                                          wheeldiameter,

                                          TunerConstants9998::kDriveGearRatio,
                                          1.0,
                                          TunerConstants9998::kSteerGearRatio,
                                          TunerConstants9998::kSpeedAt12Volts,

                                          TunerConstants9998::kFrontRightDriveMotorId,
                                          TunerConstants9998::kInvertRightSide,

                                          TunerConstants9998::kFrontRightSteerMotorId,
                                          TunerConstants9998::kFrontRightSteerMotorInverted,

                                          TunerConstants9998::kFrontRightEncoderId,
                                          TunerConstants9998::kFrontRightEncoderInverted,
                                          TunerConstants9998::kFrontRightEncoderOffset,

                                          TunerConstants9998::steerGains,

                                          networkTableName);

    m_rightBackModule = new SwerveModule(canbusName,
                                         SwerveModuleConstants::ModuleID::RIGHT_BACK,
                                         wheeldiameter,

                                         TunerConstants9998::kDriveGearRatio,
                                         1.0,
                                         TunerConstants9998::kSteerGearRatio,
                                         TunerConstants9998::kSpeedAt12Volts,

                                         TunerConstants9998::kBackRightDriveMotorId,
                                         TunerConstants9998::kInvertRightSide,

                                         TunerConstants9998::kBackRightSteerMotorId,
                                         TunerConstants9998::kBackRightSteerMotorInverted,

                                         TunerConstants9998::kBackRightEncoderId,
                                         TunerConstants9998::kBackRightEncoderInverted,
                                         TunerConstants9998::kBackRightEncoderOffset,

                                         TunerConstants9998::steerGains,

                                         networkTableName);

    units::length::inch_t wheelbase = TunerConstants9998::kFrontLeftXPos -
                                      TunerConstants9998::kBackLeftXPos;
    units::length::inch_t trackwidth = TunerConstants9998::kFrontLeftYPos -
                                       TunerConstants9998::kFrontRightYPos;

    m_chassis = new SwerveChassis(m_leftFrontModule,
                                  m_rightFrontModule,
                                  m_leftBackModule,
                                  m_rightBackModule,
                                  m_pigeon2,
                                  networkTableName,
                                  wheelbase,
                                  trackwidth,
                                  wheeldiameter,
                                  TunerConstants9998::kSpeedAt12Volts);
}
