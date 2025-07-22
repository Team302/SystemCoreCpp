#pragma once

#include "ctre/phoenix6/swerve/SwerveDrivetrain.hpp"

using namespace ctre::phoenix6;
/**
 * \brief Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types.
 */
class TunerSwerveDrivetrain : public swerve::SwerveDrivetrain<hardware::TalonFX, hardware::TalonFX, hardware::CANcoder>
{
public:
    using SwerveModuleConstants = swerve::SwerveModuleConstants<configs::TalonFXConfiguration, configs::TalonFXConfiguration, configs::CANcoderConfiguration>;

    /**
     * \brief Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them
     * through getters in the classes.
     *
     * \param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * \param modules             Constants for each specific module
     */
    template <std::same_as<SwerveModuleConstants>... ModuleConstants>
    TunerSwerveDrivetrain(swerve::SwerveDrivetrainConstants const &driveTrainConstants, ModuleConstants const &...modules) : SwerveDrivetrain{driveTrainConstants, modules...}
    {
    }

    /**
     * \brief Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them
     * through getters in the classes.
     *
     * \param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * \param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * \param modules                    Constants for each specific module
     */
    template <std::same_as<SwerveModuleConstants>... ModuleConstants>
    TunerSwerveDrivetrain(
        swerve::SwerveDrivetrainConstants const &driveTrainConstants,
        units::hertz_t odometryUpdateFrequency,
        ModuleConstants const &...modules) : SwerveDrivetrain{driveTrainConstants, odometryUpdateFrequency, modules...}
    {
    }

    /**
     * \brief Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them
     * through getters in the classes.
     *
     * \param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * \param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * \param odometryStandardDeviation  The standard deviation for odometry calculation
     *                                   in the form [x, y, theta]ᵀ, with units in meters
     *                                   and radians
     * \param visionStandardDeviation    The standard deviation for vision calculation
     *                                   in the form [x, y, theta]ᵀ, with units in meters
     *                                   and radians
     * \param modules                    Constants for each specific module
     */
    template <std::same_as<SwerveModuleConstants>... ModuleConstants>
    TunerSwerveDrivetrain(
        swerve::SwerveDrivetrainConstants const &driveTrainConstants,
        units::hertz_t odometryUpdateFrequency,
        std::array<double, 3> const &odometryStandardDeviation,
        std::array<double, 3> const &visionStandardDeviation,
        ModuleConstants const &...modules) : SwerveDrivetrain{
                                                 driveTrainConstants, odometryUpdateFrequency,
                                                 odometryStandardDeviation, visionStandardDeviation, modules...}
    {
    }
};
