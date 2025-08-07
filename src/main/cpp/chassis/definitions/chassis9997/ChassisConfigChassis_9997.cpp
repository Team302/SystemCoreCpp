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

#include "chassis/definitions/chassis9997/ChassisConfigChassis_9997.h"
#include "chassis/definitions/chassis9997/TunerConstants9997.h"
#include "chassis/SwerveModule.h"
#include "chassis/SwerveModuleConstants.h"

using ctre::phoenix6::configs::MountPoseConfigs;
using ctre::phoenix6::hardware::Pigeon2;
using std::string;

void ChassisConfigChassis_9997::DefinePigeon()
{
    string canbusName;
    canbusName.assign(TunerConstants9997::kCANBusName);
    m_pigeon2 = new Pigeon2(TunerConstants9997::kPigeonId, canbusName);
    MountPoseConfigs config{};
    config.MountPoseYaw = units::angle::degree_t(0.0);
    m_pigeon2->GetConfigurator().Apply(config);
    m_pigeon2->Reset();
}

void ChassisConfigChassis_9997::DefineChassis()
{
    string networkTableName{string("swerve")};
    string canbusName;
    canbusName.assign(TunerConstants9997::kCANBusName);
    units::length::inch_t wheeldiameter = TunerConstants9997::kWheelRadius * 2.0;

    m_leftFrontModule = new SwerveModule(canbusName,
                                         SwerveModuleConstants::ModuleID::LEFT_FRONT,
                                         wheeldiameter,

                                         TunerConstants9997::kDriveGearRatio,
                                         1.0,
                                         TunerConstants9997::kSteerGearRatio,
                                         TunerConstants9997::kSpeedAt12Volts,

                                         TunerConstants9997::kFrontLeftDriveMotorId,
                                         TunerConstants9997::kInvertLeftSide,

                                         TunerConstants9997::kFrontLeftSteerMotorId,
                                         TunerConstants9997::kFrontLeftSteerMotorInverted,

                                         TunerConstants9997::kFrontLeftEncoderId,
                                         TunerConstants9997::kFrontLeftEncoderInverted,
                                         TunerConstants9997::kFrontLeftEncoderOffset,

                                         TunerConstants9997::steerGains,

                                         networkTableName);

    m_leftBackModule = new SwerveModule(canbusName,
                                        SwerveModuleConstants::ModuleID::LEFT_BACK,
                                        wheeldiameter,

                                        TunerConstants9997::kDriveGearRatio,
                                        1.0,
                                        TunerConstants9997::kSteerGearRatio,
                                        TunerConstants9997::kSpeedAt12Volts,

                                        TunerConstants9997::kBackLeftDriveMotorId,
                                        TunerConstants9997::kInvertLeftSide,

                                        TunerConstants9997::kBackLeftSteerMotorId,
                                        TunerConstants9997::kBackLeftSteerMotorInverted,

                                        TunerConstants9997::kBackLeftEncoderId,
                                        TunerConstants9997::kBackLeftEncoderInverted,
                                        TunerConstants9997::kBackLeftEncoderOffset,

                                        TunerConstants9997::steerGains,

                                        networkTableName);

    m_rightFrontModule = new SwerveModule(canbusName,
                                          SwerveModuleConstants::ModuleID::RIGHT_FRONT,
                                          wheeldiameter,

                                          TunerConstants9997::kDriveGearRatio,
                                          1.0,
                                          TunerConstants9997::kSteerGearRatio,
                                          TunerConstants9997::kSpeedAt12Volts,

                                          TunerConstants9997::kFrontRightDriveMotorId,
                                          TunerConstants9997::kInvertRightSide,

                                          TunerConstants9997::kFrontRightSteerMotorId,
                                          TunerConstants9997::kFrontRightSteerMotorInverted,

                                          TunerConstants9997::kFrontRightEncoderId,
                                          TunerConstants9997::kFrontRightEncoderInverted,
                                          TunerConstants9997::kFrontRightEncoderOffset,

                                          TunerConstants9997::steerGains,

                                          networkTableName);

    m_rightBackModule = new SwerveModule(canbusName,
                                         SwerveModuleConstants::ModuleID::RIGHT_BACK,
                                         wheeldiameter,

                                         TunerConstants9997::kDriveGearRatio,
                                         1.0,
                                         TunerConstants9997::kSteerGearRatio,
                                         TunerConstants9997::kSpeedAt12Volts,

                                         TunerConstants9997::kBackRightDriveMotorId,
                                         TunerConstants9997::kInvertRightSide,

                                         TunerConstants9997::kBackRightSteerMotorId,
                                         TunerConstants9997::kBackRightSteerMotorInverted,

                                         TunerConstants9997::kBackRightEncoderId,
                                         TunerConstants9997::kBackRightEncoderInverted,
                                         TunerConstants9997::kBackRightEncoderOffset,

                                         TunerConstants9997::steerGains,

                                         networkTableName);

    units::length::inch_t wheelbase = TunerConstants9997::kFrontLeftXPos -
                                      TunerConstants9997::kBackLeftXPos;
    units::length::inch_t trackwidth = TunerConstants9997::kFrontLeftYPos -
                                       TunerConstants9997::kFrontRightYPos;

    m_chassis = new SwerveChassis(m_leftFrontModule,
                                  m_rightFrontModule,
                                  m_leftBackModule,
                                  m_rightBackModule,
                                  m_pigeon2,
                                  networkTableName,
                                  wheelbase,
                                  trackwidth,
                                  wheeldiameter,
                                  TunerConstants9997::kSpeedAt12Volts);
}
