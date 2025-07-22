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

#include "InterpolateUtils.h"
#include <cmath>
/// @brief This method performs a linear interpolation for position
///              given x and y arrays and the target x.  This assumes
///              that the x array is in ascending order.
/// @param [in] x - x array
/// @param [in] y - y array
/// @param [in] size - number of elements in the array
/// @param [in] targetX - target x value to interpolate
/// @returns units::length::meter_t  - interpolated value
units::length::meter_t InterpolateUtils::linearInterpolate(const units::length::meter_t x[],
                                                           const units::length::meter_t y[],
                                                           int size,
                                                           units::length::meter_t targetX)
{
    // Handle edge cases (targetX outside the range of x values)
    if (targetX <= x[0])
    {
        return y[0];
    }
    else if (targetX >= x[size - 1])
    {
        return y[size - 1];
    }

    // Find the indices of the x values surrounding targetX
    int i = 0;
    while (i < size - 1 && x[i] < targetX)
    {
        i++;
    }

    return units::length::meter_t(std::lerp(x[i - 1].value(), x[i].value(), targetX.value()));
}

/// @brief This method performs a linear interpolation for velocity
///              given x and y arrays and the target x.  This assumes
///              that the x array is in ascending order.
/// @param [in] x - x array
/// @param [in] y - y array
/// @param [in] size - number of elements in the array
/// @param [in] targetX - target x value to interpolate
/// @returns units::velocity::meters_per_second_t  - interpolated value
units::velocity::meters_per_second_t InterpolateUtils::linearInterpolate(
    const units::velocity::meters_per_second_t x[],
    const units::velocity::meters_per_second_t y[],
    int size,
    units::velocity::meters_per_second_t targetX)
{
    // Handle edge cases (targetX outside the range of x values)
    if (targetX <= x[0])
    {
        return y[0];
    }
    else if (targetX >= x[size - 1])
    {
        return y[size - 1];
    }

    // Find the indices of the x values surrounding targetX
    int i = 0;
    while (i < size - 1 && x[i] < targetX)
    {
        i++;
    }

    return units::velocity::meters_per_second_t(std::lerp(x[i - 1].value(), x[i].value(), targetX.value()));
}

/// @brief This method performs a linear interpolation for angle
///              given x and y arrays and the target x.  This assumes
///              that the x array is in ascending order.
/// @param [in] x - x array
/// @param [in] y - y array
/// @param [in] size - number of elements in the array
/// @param [in] targetX - target x value to interpolate
/// @returns units::angle::degree_t  - interpolated value
units::angle::degree_t InterpolateUtils::linearInterpolate(
    const units::angle::degree_t x[],
    const units::angle::degree_t y[],
    int size,
    units::angle::degree_t targetX)
{
    // Handle edge cases (targetX outside the range of x values)
    if (targetX <= x[0])
    {
        return y[0];
    }
    else if (targetX >= x[size - 1])
    {
        return y[size - 1];
    }

    // Find the indices of the x values surrounding targetX
    int i = 0;
    while (i < size - 1 && x[i] < targetX)
    {
        i++;
    }

    return units::angle::degree_t(std::lerp(x[i - 1].value(), x[i].value(), targetX.value()));
}

/// @brief This method performs a linear interpolation for voltage
///              given x and y arrays and the target x.  This assumes
///              that the x array is in ascending order.
/// @param [in] x - x array
/// @param [in] y - y array
/// @param [in] size - number of elements in the array
/// @param [in] targetX - target x value to interpolate
/// @returns units::voltage::volt_t  - interpolated value
units::voltage::volt_t InterpolateUtils::linearInterpolate(
    const units::voltage::volt_t x[],
    const units::voltage::volt_t y[],
    int size,
    units::voltage::volt_t targetX)
{
    // Handle edge cases (targetX outside the range of x values)
    if (targetX <= x[0])
    {
        return y[0];
    }
    else if (targetX >= x[size - 1])
    {
        return y[size - 1];
    }

    // Find the indices of the x values surrounding targetX
    int i = 0;
    while (i < size - 1 && x[i] < targetX)
    {
        i++;
    }

    return units::voltage::volt_t(std::lerp(x[i - 1].value(), x[i].value(), targetX.value()));
}

/// @brief This method performs a linear interpolation for angular velocity
///              given x and y arrays and the target x.  This assumes
///              that the x array is in ascending order.
/// @param [in] x - x array
/// @param [in] y - y array
/// @param [in] size - number of elements in the array
/// @param [in] targetX - target x value to interpolate
/// @returns units::angular velocity::radians_per_second_t  - interpolated value
units::angular_velocity::radians_per_second_t InterpolateUtils::linearInterpolate(
    const units::angular_velocity::radians_per_second_t x[],
    const units::angular_velocity::radians_per_second_t y[],
    int size,
    units::angular_velocity::radians_per_second_t targetX)
{
    // Handle edge cases (targetX outside the range of x values)
    if (targetX <= x[0])
    {
        return y[0];
    }
    else if (targetX >= x[size - 1])
    {
        return y[size - 1];
    }

    // Find the indices of the x values surrounding targetX
    int i = 0;
    while (i < size - 1 && x[i] < targetX)
    {
        i++;
    }

    return units::angular_velocity::radians_per_second_t(std::lerp(x[i - 1].value(), x[i].value(), targetX.value()));
}

/// @brief This method performs a linear interpolation for double
///              given x and y arrays and the target x.  This assumes
///              that the x array is in ascending order.
/// @param [in] x - x array
/// @param [in] y - y array
/// @param [in] size - number of elements in the array
/// @param [in] targetX - target x value to interpolate
/// @returns double  - interpolated value
double InterpolateUtils::linearInterpolate(
    const double x[],
    const double y[],
    int size,
    double targetX)
{
    // Handle edge cases (targetX outside the range of x values)
    if (targetX <= x[0])
    {
        return y[0];
    }
    else if (targetX >= x[size - 1])
    {
        return y[size - 1];
    }

    // Find the indices of the x values surrounding targetX
    int i = 0;
    while (i < size - 1 && x[i] < targetX)
    {
        i++;
    }

    return std::lerp(x[i - 1], x[i], targetX);
}