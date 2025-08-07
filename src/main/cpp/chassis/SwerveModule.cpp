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

// C++ Includes
#include <memory>
#include <numbers>
#include <string>

// FRC includes
#include "frc/Filesystem.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "units/angle.h"
#include "units/length.h"
#include "units/velocity.h"
#include "frc/RobotBase.h"

// Team 302 includes
#include "chassis/SwerveChassis.h"
#include "chassis/SwerveModule.h"
#include "chassis/SwerveModuleConstants.h"
#include "utils/AngleUtils.h"
#include "utils/logging/debug/Logger.h"
#include "utils/DragonPower.h"

// Third Party Includes
#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/TalonFX.hpp"
#include "pugixml/pugixml.hpp"
#include "units/current.h"
#include "units/voltage.h"

using namespace std;
using namespace frc;

using std::string;

using ctre::phoenix6::configs::CANcoderConfiguration;
using ctre::phoenix6::configs::CANcoderConfigurator;
using ctre::phoenix6::configs::CurrentLimitsConfigs;
using ctre::phoenix6::configs::MotorOutputConfigs;
using ctre::phoenix6::configs::Slot0Configs;
using ctre::phoenix6::configs::TalonFXConfiguration;
using ctre::phoenix6::configs::VoltageConfigs;
using ctre::phoenix6::controls::DutyCycleOut;
using ctre::phoenix6::controls::PositionVoltage;
using ctre::phoenix6::controls::VelocityTorqueCurrentFOC;
using ctre::phoenix6::hardware::CANcoder;
using ctre::phoenix6::hardware::TalonFX;
using ctre::phoenix6::signals::FeedbackSensorSourceValue;
using ctre::phoenix6::signals::InvertedValue;
using ctre::phoenix6::signals::NeutralModeValue;
using ctre::phoenix6::signals::SensorDirectionValue;

using namespace ctre::phoenix6;
//==================================================================================
SwerveModule::SwerveModule(std::string canbusname,
                           SwerveModuleConstants::ModuleID id,
                           units::length::inch_t wheelDiameter,
                           units::dimensionless::scalar_t driveGearRatio,
                           double sensorToMechanismRatio,
                           units::dimensionless::scalar_t rotorToSensorRatio,
                           units::velocity::feet_per_second_t maxSpeed,
                           int driveMotorID,
                           bool driveInverted,
                           int turnMotorID,
                           bool turnInverted,
                           int canCoderID,
                           bool canCoderInverted,
                           const units::angle::turn_t angleOffset,
                           ctre::phoenix6::configs::Slot0Configs steerGains,
                           std::string networkTableName) : m_moduleID(id),
                                                           m_driveTalon(new TalonFX(driveMotorID, canbusname)),
                                                           m_steerTalon(new TalonFX(turnMotorID, canbusname)),
                                                           m_steerCancoder(new CANcoder(canCoderID, canbusname)),
                                                           m_activeState(),
                                                           m_wheelDiameter(wheelDiameter),
                                                           m_maxSpeed(maxSpeed),
                                                           m_steerGains(steerGains),
                                                           m_networkTableName(networkTableName)

{

    Rotation2d ang{units::angle::degree_t(0.0)};
    m_activeState.angle = ang;
    m_activeState.speed = 0_mps;
    m_simDriveDistance = 0_m;
    m_simDriveSpeed = 0_mps;

    m_wheelDiameter = wheelDiameter;
    m_maxSpeed = maxSpeed;
    m_gearRatio = driveGearRatio;

    // ReadConstants(configfilename);
    InitDriveMotor(driveInverted);
    InitSteerMotorEncoder(turnInverted, canCoderInverted, angleOffset, sensorToMechanismRatio, rotorToSensorRatio);

    m_moduleConfig.wheelRadius = GetWheelDiameter() / 2.0;
    m_moduleConfig.maxDriveVelocityMPS = GetMaxSpeed();
    m_moduleConfig.driveCurrentLimit = GetDriveCurrentLimit();
    m_moduleConfig.driveMotor = GetDriveMotorDef();
    m_moduleConfig.wheelCOF = GetCoefficientOfFriction();
}

//==================================================================================
/// @brief Get the encoder values
/// @returns double - the integrated sensor position
double SwerveModule::GetEncoderValues()
{
    return m_driveTalon->GetPosition().GetValueAsDouble() * 2048;
}

//==================================================================================
/// @brief Turn all of the wheel to zero degrees yaw according to the pigeon
/// @returns void
void SwerveModule::ZeroAlignModule()
{
    SetSteerAngle(units::degree_t(0));
}

//==================================================================================
/// @brief Get the current state of the module (speed of the wheel and angle of the wheel)
/// @returns SwerveModuleState
SwerveModuleState SwerveModule::GetState() const
{
    // Get the Module Drive Motor Speed
    auto mpr = units::length::meter_t(GetWheelDiameter() * numbers::pi);
    units::velocity::meters_per_second_t mps = units::velocity::meters_per_second_t(0.0);
    mps = units::velocity::meters_per_second_t(mpr.to<double>() * m_driveTalon->GetVelocity().GetValueAsDouble());

    // Get the Module Current Rotation Angle
    units::angle::degree_t ang = m_steerCancoder->GetAbsolutePosition().GetValue();
    Rotation2d angle = Rotation2d(ang);

    // Create the state and return it
    SwerveModuleState state{mps, angle};
    return state;
}

//==================================================================================
/// @brief Get the current position of the swerve module (distance and rotation)
/// @return frc::SwerveModulePosition - current position
frc::SwerveModulePosition SwerveModule::GetPosition()
{
    double rotations = 0.0;
    rotations = m_driveTalon->GetPosition().GetValueAsDouble();
    units::angle::degree_t angle = m_steerCancoder->GetAbsolutePosition().GetValue();
    Rotation2d currAngle = Rotation2d(angle);

    if (frc::RobotBase::IsSimulation())
    {
        m_simDriveDistance = m_simDriveDistance + (m_simDriveSpeed * 0.02_s);
        currAngle = Rotation2d(m_activeState.angle.Degrees());
        return {m_simDriveDistance, currAngle};
    }

    return {rotations * m_wheelDiameter * numbers::pi, // distance travled by drive motor
            currAngle};                                // angle of the swerve module from sensor
}

//==================================================================================
/// @brief Set the current state of the module (speed of the wheel and angle of the wheel)
/// @param [in] const SwerveModuleState& targetState:   state to set the module to
/// @returns void
void SwerveModule::SetDesiredState(SwerveModuleState &targetState)
{
    // Update targets so the angle turned is less than 90 degrees
    // If the desired angle is less than 90 degrees from the target angle (e.g., -90 to 90 is the amount of turn), just use the angle and speed values
    // if it is more than 90 degrees (90 to 270), the can turn the opposite direction -- increase the angle by 180 degrees -- and negate the wheel speed
    // finally, get the value between -90 and 90

    units::angle::degree_t angle = m_steerCancoder->GetAbsolutePosition().GetValue();
    Rotation2d currAngle = Rotation2d(angle);
    targetState.Optimize(currAngle);

    //   Set Steer Target
    SetSteerAngle(targetState.angle.Degrees());

    // Set Drive Target
    SetDriveSpeed(targetState.speed);
}

//==================================================================================
/// @brief Run the swerve module at the same speed and angle
/// @returns void
void SwerveModule::RunCurrentState()
{
    SetDriveSpeed(m_activeState.speed);
}

//==================================================================================
/// @brief run the drive motor at a specified speed
/// @param [in] speed to drive the drive wheel as
/// @returns void
void SwerveModule::SetDriveSpeed(units::velocity::meters_per_second_t speed)
{
    // TODO come back to the activate speed option
    // m_activeState.speed = (abs(speed.to<double>() / m_maxSpeed.to<double>()) < 0.05) ? 0_mps : speed;
    // m_activeState.speed = units::velocity::meters_per_second_t(std::clamp(m_activeState.speed.value(), -m_maxSpeed.value(), m_maxSpeed.value()));
    // if (m_activeState.speed != 0_mps)
    m_activeState.speed = speed;
    if (m_activeState.speed != 0_mps)
    {

        if (m_velocityControlled)
        {
            units::angular_velocity::turns_per_second_t omega = units::angular_velocity::turns_per_second_t(m_activeState.speed.value() / (numbers::pi * units::length::meter_t(m_wheelDiameter).value())); // convert mps to unitless rps by taking the speed and dividing by the circumference of the wheel
            if (m_useFOC)
                m_driveTalon->SetControl(m_velocityTorque.WithVelocity(omega));
            else
                m_driveTalon->SetControl(m_velocityVoltage.WithVelocity(omega));
        }
        else // Run Open Loop
        {
            auto percent = m_activeState.speed / m_maxSpeed;
            DutyCycleOut out{percent};
            m_driveTalon->SetControl(out);
        }
    }
    else
        m_driveTalon->SetControl(ctre::phoenix6::controls::NeutralOut());

    m_simDriveSpeed = m_activeState.speed;
}

//==================================================================================
/// @brief Turn the swerve module to a specified angle
/// @param [in] units::angle::degree_t the target angle to turn the wheel to
/// @returns void
void SwerveModule::SetSteerAngle(units::angle::degree_t targetAngle)
{
    m_activeState.angle = targetAngle;
    units::angle::turn_t targetAngleTurns = targetAngle;

    // TODO come back to non-foc implementation
    // if (m_useFOC)
    m_steerTalon->SetControl(m_positionVoltage.WithPosition(targetAngleTurns));
    // else
    //     m_steerTalon->SetControl(m_positionTorque.WithPosition(targetAngleTurns));
}
//==================================================================================

/**
 * Get real speed of module
 * @param inertialVelocity Inertial velocity of robot (m/s)
 * @param rotateRate Rotate rate of robot (rad/s)
 * @param radius Radius of swerve drive (center to the furtherest wheel)
 * @return Speed of module (m/s)
 */
units::velocity::meters_per_second_t SwerveModule::CalculateRealSpeed(units::velocity::meters_per_second_t inertialVelocity, units::angular_velocity::radians_per_second_t rotateRate, units::length::meter_t radius)
{
    return units::velocity::meters_per_second_t(inertialVelocity.value() + rotateRate.value() * radius.value());
}

//==================================================================================
/// @brief stop the drive and turn motors
/// @return void
void SwerveModule::StopMotors()
{
    SetDriveSpeed(0_mps);
}

//==================================================================================
void SwerveModule::LogInformation()
{
    string ntAngleName;
    string ntMotorPositionName;
    string ntRotorPositionName;
    string ntName;
    if (m_moduleID == SwerveModuleConstants::ModuleID::LEFT_BACK)
    {
        ntAngleName += string("Left Back Angle");
        ntMotorPositionName += string("Left Back Steer");
        ntRotorPositionName += string("Left Back Drive");
        ntName = string("Left Back Swerve Encoders");
    }
    else if (m_moduleID == SwerveModuleConstants::ModuleID::LEFT_FRONT)
    {
        ntAngleName += string("Left Front Angle");
        ntMotorPositionName += string("Left Front Steer");
        ntRotorPositionName += string("Left Front Drive");
        ntName = string("Left Front Swerve Encoders");
    }
    else if (m_moduleID == SwerveModuleConstants::ModuleID::RIGHT_BACK)
    {
        ntAngleName += string("Right Back Angle");
        ntMotorPositionName += string("Right Back Steer");
        ntRotorPositionName += string("Right Back Drive");
        ntName = string("Right Back Swerve Encoders");
    }
    else
    {
        ntAngleName += string("Right Front Angle");
        ntMotorPositionName += string("Right Front Steer");
        ntRotorPositionName += string("Right Front Drive");
        ntName = string("Right Front Swerve Encoders");
    }
    auto turns = m_steerTalon->GetPosition().GetValueAsDouble();
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, ntName, ntMotorPositionName, turns);

    auto rotor = m_steerTalon->GetRotorPosition().GetValueAsDouble();
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, ntName, ntRotorPositionName, rotor);
}

//==================================================================================
void SwerveModule::InitDriveMotor(bool driveInverted)
{
    if (m_driveTalon != nullptr)
    {
        m_driveTalon->GetConfigurator().Apply(TalonFXConfiguration{}); // Apply Factory Defaults

        configs::TalonFXConfiguration configs{};

        configs.Voltage.PeakForwardVoltage = units::volt_t(10.0);
        configs.Voltage.PeakReverseVoltage = units::volt_t(-10.0);

        /// TO DO : Need code gen updates to be able to be implemented
        configs.Slot0.kS = m_driveKs; // To account for friction, static feedforward
        configs.Slot0.kV = m_driveKf; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second(Free wheeling)
        configs.Slot0.kP = m_driveKp;
        configs.Slot0.kI = m_driveKi;
        configs.Slot0.kD = m_driveKd;
        configs.MotorOutput.NeutralMode = NeutralModeValue::Brake;
        configs.MotorOutput.Inverted = driveInverted ? InvertedValue::Clockwise_Positive : InvertedValue::CounterClockwise_Positive;
        configs.MotorOutput.PeakForwardDutyCycle = 1.0;
        configs.MotorOutput.PeakReverseDutyCycle = -1.0;
        configs.MotorOutput.DutyCycleNeutralDeadband = 0.0;

        configs.CurrentLimits.StatorCurrentLimit = units::ampere_t(80.0);
        configs.CurrentLimits.StatorCurrentLimitEnable = true;
        configs.CurrentLimits.SupplyCurrentLimit = units::ampere_t(60.0);
        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        configs.CurrentLimits.SupplyCurrentLowerLimit = units::ampere_t(80.0); // TODO Double check
        configs.CurrentLimits.SupplyCurrentLowerTime = units::second_t(0.15);  // TODO Double check
        configs.Feedback.SensorToMechanismRatio = m_gearRatio;

        /* Retry config apply up to 5 times, report if failure */
        ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i)
        {
            status = m_driveTalon->GetConfigurator().Apply(configs);
            if (status.IsOK())
                break;
        }
        if (!status.IsOK())
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, "SwerveModule::InitDriveMotor", string("Could not apply configs, error code"), status.GetName());
        }
        m_driveTalon->SetPosition(0_tr);
    }
}

//==================================================================================
void SwerveModule::InitSteerMotorEncoder(bool turnInverted,
                                         bool canCoderInverted,
                                         const units::angle::turn_t angleOffset,
                                         double sensorToMechanismRatio,
                                         units::dimensionless::scalar_t rotorToSensorRatio)
{
    if (m_steerTalon != nullptr && m_steerCancoder != nullptr)
    {
        m_steerTalon->GetConfigurator().Apply(TalonFXConfiguration{}); // Apply Factory Defaults

        TalonFXConfiguration fxconfigs{};
        m_steerTalon->GetConfigurator().Refresh(fxconfigs);

        fxconfigs.MotorOutput.Inverted = turnInverted ? InvertedValue::Clockwise_Positive : InvertedValue::Clockwise_Positive;
        fxconfigs.MotorOutput.NeutralMode = NeutralModeValue::Brake;
        fxconfigs.MotorOutput.PeakForwardDutyCycle = 1.0;
        fxconfigs.MotorOutput.PeakReverseDutyCycle = -1.0;
        fxconfigs.MotorOutput.DutyCycleNeutralDeadband = 0.0;

        fxconfigs.Voltage.PeakForwardVoltage = units::volt_t(10.0);
        fxconfigs.Voltage.PeakReverseVoltage = units::volt_t(-10.0);

        // Peak output of 120 amps
        fxconfigs.TorqueCurrent.PeakForwardTorqueCurrent = units::ampere_t(120.0);  // TODO check this
        fxconfigs.TorqueCurrent.PeakReverseTorqueCurrent = units::ampere_t(-120.0); // TODO check this

        fxconfigs.CurrentLimits.SupplyCurrentLimit = units::ampere_t(60.0);
        fxconfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        fxconfigs.Slot0 = m_steerGains;

        fxconfigs.ClosedLoopGeneral.ContinuousWrap = true;

        fxconfigs.Feedback.FeedbackRemoteSensorID = m_steerCancoder->GetDeviceID();
        fxconfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue::FusedCANcoder;
        fxconfigs.Feedback.SensorToMechanismRatio = sensorToMechanismRatio;
        fxconfigs.Feedback.RotorToSensorRatio = rotorToSensorRatio;

        ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i)
        {
            status = m_steerTalon->GetConfigurator().Apply(fxconfigs);
            if (status.IsOK())
                break;
        }
        if (!status.IsOK())
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, "SwerveModule::InitSteerMotorEncoder", string("Could not apply configs, error code"), status.GetName());
        }

        m_steerTalon->SetPosition(0_tr);

        CANcoderConfiguration ccConfigs{};
        m_steerCancoder->GetConfigurator().Apply(ccConfigs); // Apply Factory Defaults

        ccConfigs.MagnetSensor.MagnetOffset = units::angle::degree_t(angleOffset);
        ccConfigs.MagnetSensor.SensorDirection = canCoderInverted ? SensorDirectionValue::Clockwise_Positive : SensorDirectionValue::CounterClockwise_Positive;
        ccConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = units::degree_t(180.0);

        m_steerCancoder->GetConfigurator().Apply(ccConfigs);
    }
}

std::tuple<double, double> SwerveModule::CalcSteerPowerEnergy(units::time::second_t deltaTime)
{
    return DragonPower::CalcPowerEnergy(deltaTime, m_steerTalon->GetSupplyVoltage().GetValueAsDouble(), m_steerTalon->GetSupplyCurrent().GetValueAsDouble());
}
std::tuple<double, double> SwerveModule::CalcDrivePowerEnergy(units::time::second_t deltaTime)
{
    return DragonPower::CalcPowerEnergy(deltaTime, m_driveTalon->GetSupplyVoltage().GetValueAsDouble(), m_driveTalon->GetSupplyCurrent().GetValueAsDouble());
}
