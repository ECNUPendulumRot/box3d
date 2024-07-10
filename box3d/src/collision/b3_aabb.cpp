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


#include "collision/b3_aabb.hpp"

/**
 * @brief constructor of b3AABB
 * @param lower_bound lower bound of the b3AABB
 * @param upper_bound upper bound of the b3AABB
 * @attention The lower bound must be less than the upper bound.
 */
b3AABB::b3AABB(const b3Vec3r &lower_bound, const b3Vec3r &upper_bound) {
    m_min = lower_bound;
    m_max = upper_bound;
}

/**
 * @brief Check whether 2 AABBs are overlapped.
 * @param A: The first AABB.
 * @param B: The second AABB.
 * @details This method is introduced in @cite Real-Time Collision Detection, Chapter 4
 * @return True if 2 AABBs are overlapped, false otherwise.
 */
bool b3AABB::overlapped(const b3AABB &A, const b3AABB &B)
{
    if (A.m_max.x < B.m_min.x || A.m_min.x > B.m_max.x) return false;
    if (A.m_max.y < B.m_min.y || A.m_min.y > B.m_max.y) return false;
    if (A.m_max.z < B.m_min.z || A.m_min.z > B.m_max.z) return false;

    return true;
}


