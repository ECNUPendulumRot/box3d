// The MIT License

// Copyright (c) 2024
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#ifndef BOX3D_B3_COMMON_HPP
#define BOX3D_B3_COMMON_HPP


#include <cassert>
#include <cfloat>
#include <cmath>

#include "common/b3_types.hpp"


#define b3_assert(A) assert(A) ///< This macro redefines the assert function from the C standard library to be used as b3_assert.

#define b3_pi 3.14159265359 ///< This macro defines the constant b3_pi to represent the value of π (pi),

#define b3_close_to_zero_epsilon 1e-5 ///< It is used to determine if a floating-point number is close enough to zero to be considered zero.

#define b3_NOT_USED(x) ((void)(x)) ///< This macro casts the variable x to void, effectively suppressing compiler warnings about unused variables.

#define b3_length_units_per_meter 1.0 ///< Used to maintain consistency in length units throughout the code.

#define b3_linear_slop  (0.005 * b3_length_units_per_meter) ///< Used as a tolerance value in physical simulations or geometric calculations to prevent issues with floating-point precision.g

#define b3_max_linear_correction (0.2 * b3_length_units_per_meter) ///< Used in constraint solvers or collision response to limit the maximum correction applied to overlapping objects.

#define b3_polygon_radius (2.0 * b3_linear_slop)///<  Used in collision detection algorithms to define a small buffer radius around polygons.

#define b3_aabb_extension (0.1 * b3_length_units_per_meter) ///< Used to extend the bounds of AABBs for broad-phase collision detection to account for motion or numerical inaccuracies.

#define b3_linear_sleep_tolerance		(0.01 * b3_length_units_per_meter) ///< A body cannot sleep if its linear velocity is above this tolerance.

#define b3_angular_sleep_tolerance	(2.0 / 180.0 * b3_pi) ///< A body cannot sleep if its angular velocity is above this tolerance.

#define b3_baumgarte 0.2 ///< This macro defines the Baumgarte stabilization factor, b3_baumgarte, which is set to 0.2.

#define b3_max_sub_steps 8 ///< This macro defines the maximum number of sub-steps, b3_max_sub_steps, which is set to 8.

using real = float; ///<  It defines real as an alias for the float data type.

/**
 * @brief This is the name of the template struct being declared.
 * @tparam T This is a template declaration, indicating that b3RealLimits is a template
 * that takes one type parameter T.
 */
template<typename T>
struct b3RealLimits;

/**
 * @brief  It defines the maximum, minimum, and epsilon values for single-precision
 * floating-point numbers.
 */
template<>
struct b3RealLimits<float> {
    /**
     * @brief Used to get the largest possible float value.
     */
    static constexpr float Max = FLT_MAX;

    /**
     * @brief  Used to get the smallest positive normalized float value.
     */
    static constexpr float Min = FLT_MIN;

    /**
     * @brief Used to get the smallest increment for float precision
     */
    static constexpr float Epsilon = FLT_EPSILON;
};

/**
 * @brief  It is used to define the maximum, minimum, and epsilon values for double
 * precision floating-point numbers.
 */
template<>
struct b3RealLimits<double> {
    /**
     * @brief Used to get the largest possible double value.
     */
    static constexpr double Max = DBL_MAX;

    /**
     * @brief Used to get the smallest positive normalized double value.
     */
    static constexpr double Min = DBL_MIN;

    /**
     * @brief Used to get the smallest increment for double precision,
     * which is useful for numerical comparisons and precision checks.
     */
    static constexpr double Epsilon = DBL_EPSILON;
};

constexpr auto b3_real_max = b3RealLimits<real>::Max; ///< Defines the maximum representable value for the type real.
constexpr auto b3_real_min = b3RealLimits<real>::Min; ///< Defines the minimum positive normalized value for the type real.
constexpr auto b3_real_epsilon = b3RealLimits<real>::Epsilon; ///< Defines the smallest representable difference between two values for the type real.

/**
 * @brief This function checks if a given floating-point number is effectively zero by
 * comparing it to a small threshold value.
 * @param x The input parameter x of type real.
 * @return true: If the absolute value of x is less than the defined small threshold b3_real_epsilon.
 * false: Otherwise.
 */
inline bool b3_is_zero(real x) {
    return fabs(x) < b3_real_epsilon;
}

/**
 * @brief This function checks if a given floating-point number is close enough to
 * zero by comparing it to a small threshold value defined as b3_close_to_zero_epsilon.
 * @param x A floating-point number that needs to be checked if it is close to zero.
 * @return true: If the absolute value of x is less than the defined small threshold b3_close_to_zero_epsilon.
 * false: Otherwise.
 */
inline bool b3_close_to_zero(real x) {
    return fabs(x) < b3_close_to_zero_epsilon;
}


#endif //BOX3D_B3_COMMON_HPP
