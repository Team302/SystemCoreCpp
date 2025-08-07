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

#include "chassis/definitions/chassis9999/ChassisConfigChassis_9999.h"
#include "chassis/definitions/chassis9999/TunerConstants9999.h"
#include "chassis/SwerveModule.h"
#include "chassis/SwerveModuleConstants.h"
#include "utils/logging/debug/Logger.h"

using ctre::phoenix6::configs::MountPoseConfigs;
using ctre::phoenix6::hardware::Pigeon2;
using std::string;

void ChassisConfigChassis_9999::DefinePigeon()
{
    string canbusName;
    canbusName.assign(TunerConstants9999::kCANBusName);
    m_pigeon2 = new Pigeon2(TunerConstants9999::kPigeonId, canbusName);
    m_pigeon2->Reset();
}

void ChassisConfigChassis_9999::DefineChassis()
{
    string networkTableName{string("swerve")};
    string canbusName;
    canbusName.assign(TunerConstants9999::kCANBusName);
    units::length::inch_t wheeldiameter = TunerConstants9999::kWheelRadius * 2.0;

    m_leftFrontModule = new SwerveModule(canbusName,
                                         SwerveModuleConstants::ModuleID::LEFT_FRONT,
                                         wheeldiameter,

                                         TunerConstants9999::kDriveGearRatio,
                                         1.0,
                                         TunerConstants9999::kSteerGearRatio,
                                         TunerConstants9999::kSpeedAt12Volts,

                                         TunerConstants9999::kFrontLeftDriveMotorId,
                                         TunerConstants9999::kInvertLeftSide,

                                         TunerConstants9999::kFrontLeftSteerMotorId,
                                         TunerConstants9999::kFrontLeftSteerMotorInverted,

                                         TunerConstants9999::kFrontLeftEncoderId,
                                         TunerConstants9999::kFrontLeftEncoderInverted,
                                         TunerConstants9999::kFrontLeftEncoderOffset,

                                         TunerConstants9999::steerGains,

                                         networkTableName);

    m_leftBackModule = new SwerveModule(canbusName,
                                        SwerveModuleConstants::ModuleID::LEFT_BACK,
                                        wheeldiameter,

                                        TunerConstants9999::kDriveGearRatio,
                                        1.0,
                                        TunerConstants9999::kSteerGearRatio,
                                        TunerConstants9999::kSpeedAt12Volts,

                                        TunerConstants9999::kBackLeftDriveMotorId,
                                        TunerConstants9999::kInvertLeftSide,

                                        TunerConstants9999::kBackLeftSteerMotorId,
                                        TunerConstants9999::kBackLeftSteerMotorInverted,

                                        TunerConstants9999::kBackLeftEncoderId,
                                        TunerConstants9999::kBackLeftEncoderInverted,
                                        TunerConstants9999::kBackLeftEncoderOffset,

                                        TunerConstants9999::steerGains,

                                        networkTableName);

    m_rightFrontModule = new SwerveModule(canbusName,
                                          SwerveModuleConstants::ModuleID::RIGHT_FRONT,
                                          wheeldiameter,

                                          TunerConstants9999::kDriveGearRatio,
                                          1.0,
                                          TunerConstants9999::kSteerGearRatio,
                                          TunerConstants9999::kSpeedAt12Volts,

                                          TunerConstants9999::kFrontRightDriveMotorId,
                                          TunerConstants9999::kInvertRightSide,

                                          TunerConstants9999::kFrontRightSteerMotorId,
                                          TunerConstants9999::kFrontRightSteerMotorInverted,

                                          TunerConstants9999::kFrontRightEncoderId,
                                          TunerConstants9999::kFrontRightEncoderInverted,
                                          TunerConstants9999::kFrontRightEncoderOffset,

                                          TunerConstants9999::steerGains,

                                          networkTableName);

    m_rightBackModule = new SwerveModule(canbusName,
                                         SwerveModuleConstants::ModuleID::RIGHT_BACK,
                                         wheeldiameter,

                                         TunerConstants9999::kDriveGearRatio,
                                         1.0,
                                         TunerConstants9999::kSteerGearRatio,
                                         TunerConstants9999::kSpeedAt12Volts,

                                         TunerConstants9999::kBackRightDriveMotorId,
                                         TunerConstants9999::kInvertRightSide,

                                         TunerConstants9999::kBackRightSteerMotorId,
                                         TunerConstants9999::kBackRightSteerMotorInverted,

                                         TunerConstants9999::kBackRightEncoderId,
                                         TunerConstants9999::kBackRightEncoderInverted,
                                         TunerConstants9999::kBackRightEncoderOffset,

                                         TunerConstants9999::steerGains,

                                         networkTableName);

    units::length::inch_t wheelbase = TunerConstants9999::kFrontLeftXPos -
                                      TunerConstants9999::kBackLeftXPos;
    units::length::inch_t trackwidth = TunerConstants9999::kFrontLeftYPos -
                                       TunerConstants9999::kFrontRightYPos;

    m_chassis = new SwerveChassis(m_leftFrontModule,
                                  m_rightFrontModule,
                                  m_leftBackModule,
                                  m_rightBackModule,
                                  m_pigeon2,
                                  networkTableName,
                                  wheelbase,
                                  trackwidth,
                                  wheeldiameter,
                                  TunerConstants9999::kSpeedAt12Volts);
}
