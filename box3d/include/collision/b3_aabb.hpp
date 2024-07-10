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


#ifndef BOX3D_B3_AABB_HPP
#define BOX3D_B3_AABB_HPP

#include "common/b3_types.hpp"
#include "math/b3_vec3.hpp"
/**
 * @brief The b3AABB class represents an Axis-Aligned Bounding Box (AABB),
 * used to define a box in three-dimensional space with minimum and maximum boundaries,
 * and provides related operations.
 */
class b3AABB {

    friend class b3CubeShape;
    friend class b3SphereShape;
    friend class b3PlaneShape;
    friend class b3DynamicTree;

    /**
     * @brief The lower bound of the AABB.
     */
    b3Vec3r m_min;

    /**
     * @brief The upper bound of the AABB.
     */
    b3Vec3r m_max;

public:

    /**
     * @brief The default constructor of b3AABB.
     */
    b3AABB() = default;

    /**
     * @brief constructor of b3AABB
     * @param lower_bound lower bound of the b3AABB
     * @param upper_bound upper bound of the b3AABB
     * @attention The lower bound must be less than the upper bound.
     */
    b3AABB(const b3Vec3r& lower_bound, const b3Vec3r& upper_bound);

    /**
     * @brief Get the center coordinate of the AABB.
     * @return The center coordinate of the AABB.
     */
    inline b3Vec3r center() const {

        return real(0.5) * (m_max - m_min);
    }

    /**
     * @brief Get the minimum coordinate of the AABB.
     * @return The minimum coordinate of the AABB.
     */
    inline b3Vec3r min() const {
        return m_min;
    }

    /**
     * @brief Get the maximum coordinate of the AABB.
     * @return The maximum coordinate of the AABB.
     */
    inline b3Vec3r max() const {
        return m_max;
    }

    /**
     * @brief Get the surface area of the AABB.
     * @return The surface area of the AABB.
     */
    inline real get_surface_area(){

        real f1 = (m_max.x - m_min.x) * (m_max.y - m_min.y);
        real f2 = (m_max.x - m_min.x) * (m_max.z - m_min.z);
        real f3 = (m_max.y - m_min.y) * (m_max.z - m_min.z);
        return real(2.0) * (f1 + f2 + f3);
    }

    /**
     * @brief  Combine 2 AABBs into 1 AABB.
     * @param A one of the AABBs
     * @param B one of the AABBs
     */
    inline void combine(const b3AABB& A, const b3AABB& B) {

        m_min = b3_min_coeff(A.m_min, B.m_min);
        m_max = b3_max_coeff(A.m_max, B.m_max);
    }

    /**
     * @brief Check this AABB contains another AABB.
     * @param other potentially contained AABB.
     * @return Returns true if this AABB completely contains the other AABB.
     * Returns false if it partially contains or does not contain the other AABB at all.
     */
    inline bool contains(const b3AABB& other) const {

        bool result = true;

        result = result && m_min.x <= other.m_min.x;
        result = result && m_min.y <= other.m_min.y;
        result = result && m_min.z <= other.m_min.z;

        result = result && m_max.x >= other.m_max.x;
        result = result && m_max.y >= other.m_max.y;
        result = result && m_max.z >= other.m_max.z;

        return result;
    }

    /**
     * @brief Check whether 2 AABBs are overlapped.
     * @param A: The first AABB.
     * @param B: The second AABB.
     * @details This method is introduced in @cite Real-Time Collision Detection, Chapter 4
     * @return True if 2 AABBs are overlapped, false otherwise.
     */
    static bool overlapped(const b3AABB& A, const b3AABB& B);

};


#endif //BOX3D_B3_AABB_HPP
