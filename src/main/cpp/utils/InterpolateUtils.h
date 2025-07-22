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

#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

class InterpolateUtils
{
public:
    // Position interpolation
    units::length::meter_t linearInterpolate(const units::length::meter_t x[], const units::length::meter_t y[], int size, units::length::meter_t targetX);

    // Velocity interpolation
    units::velocity::meters_per_second_t linearInterpolate(const units::velocity::meters_per_second_t x[], const units::velocity::meters_per_second_t y[], int size, units::velocity::meters_per_second_t targetX);

    // Angle interpolation
    units::angle::degree_t linearInterpolate(const units::angle::degree_t x[], const units::angle::degree_t y[], int size, units::angle::degree_t targetX);

    // Angular Velocity interpolation
    units::angular_velocity::radians_per_second_t linearInterpolate(const units::angular_velocity::radians_per_second_t x[], const units::angular_velocity::radians_per_second_t y[], int size, units::angular_velocity::radians_per_second_t targetX);

    // Voltage interpolation
    units::voltage::volt_t linearInterpolate(const units::voltage::volt_t x[], const units::voltage::volt_t y[], int size, units::voltage::volt_t targetX);

    // Double interpolation
    double linearInterpolate(const double x[], const double y[], int size, double targetX);
};
