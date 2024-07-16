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


// This file is same as the one of bullet3
// for more information, please access:
// http://bulletphysics.org

#ifndef BOX3D_B3_MIN_MAX_HPP
#define BOX3D_B3_MIN_MAX_HPP

/**
 * @brief Finds and returns the minimum of two values.
 * @param a The first input value.
 * @param b The second input value.
 * @return A reference to the smaller of the two values, a or b.
 */
template <class T>
inline const T& b3_min(const T& a, const T& b)
{
    return a < b ? a : b;
}

/**
 * @brief Finds and returns the maximum of two values.
 * @param a The first input value.
 * @param b The second input value.
 * @return A reference to the larger of the two values, a or b.
 */
template <class T>
inline const T& b3_max(const T& a, const T& b)
{
    return a > b ? a : b;
}

/**
 * @brief Clamps a value within a specified range.
 * @param a  The input value to be clamped.
 * @param lb The lower bound of the range.
 * @param ub The upper bound of the range.
 * @return A reference to a if it is within the range [lb, ub]. If a is less than lb,
 * it returns lb. If a is greater than ub, it returns ub.
 */
template <class T>
inline const T& b3_clamp(const T& a, const T& lb, const T& ub)
{
    return a < lb ? lb : (ub < a ? ub : a);
}


#endif //BOX3D_B3_MIN_MAX_HPP
