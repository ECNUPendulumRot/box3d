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


#ifndef BOX3D_B3_MATH_OP_HPP
#define BOX3D_B3_MATH_OP_HPP


#include <cmath>
#include "common/b3_common.hpp"

/**
 * @brief Computes the square root of a given number x.
 * @param x The input value for which the square root is to be calculated.
 * The type T can be float or double.
 * @return The square root of x.
 */
template <typename T>
inline T b3_sqrt(T x) {
    if constexpr (std::is_same_v<T, double>)
        return sqrt(x);
    else
        return sqrtf(x);
}

/**
 * @brief Computes the absolute value of a given number x.
 * @param x The input value for which the absolute value is to be calculated.
 * The type T can be any numerical type.
 * @return The absolute value of x.
 */
template <typename T>
inline T b3_abs(T x) {
    return x > 0 ? x : -x;
}

/**
 * @brief Computes the sine of a given angle x.
 * @param x The input angle in radians for which the sine is to be calculated. The type T
 * can be float or double.
 * @return The sine of x.
 */
template <typename T>
inline T b3_sin(T x) {
    if constexpr (std::is_same_v<T, double>)
        return sin(x);
    else
        return sinf(x);
}

/**
 * @brief Computes the cosine of a given angle x.
 * @param x  The input angle in radians for which the cosine is to be calculated. The type T
 * can be float or double.
 * @return The cosine of x.
 */
template <typename T>
inline T b3_cos(T x) {
    if constexpr (std::is_same_v<T, double>)
        return cos(x);
    else
        return cosf(x);
}

/**
 * @brief Rounds the value of x to zero if it is less than a small epsilon value,
 * effectively eliminating very small numerical errors.
 * @param x The input value to be checked and possibly rounded to zero.
 */
inline void b3_round_to_zero(real& x) {
    x = b3_abs(x) < b3_real_epsilon ? real(0) : x;
}

#endif //BOX3D_B3_MATH_OP_HPP
