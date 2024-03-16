
#ifndef BOX3D_B3_AABB_HPP
#define BOX3D_B3_AABB_HPP

#include "common/b3_types.hpp"


class b3AABB {

    friend class b3CubeShape;
    friend class b3SphereShape;
    friend class b3PlaneShape;
    friend class b3DynamicTree;

    /**
     * The lower bound of the AABB.
     */
    b3Vector3r m_min;

    /**
     * The upper bound of the AABB.
     */
    b3Vector3r m_max;

public:

    b3AABB() = default;

    b3AABB(const b3Vector3r& lower_bound, const b3Vector3r& upper_bound);

    /**
     * @brief Get the center coordinate of the AABB.
     * @return The center coordinate of the AABB.
     */
    inline b3Vector3r center() const {

        return real(0.5) * (m_max - m_min);
    }

    inline b3Vector3r min() const {
        return m_min;
    }

    inline b3Vector3r max() const {
        return m_max;
    }

    /**
     * @brief Get the surface area of the AABB.
     * @return The surface area of the AABB.
     */
    inline real get_surface_area(){

        real f1 =  (m_max.m_x - m_min.m_x) * (m_max.m_y - m_min.m_y);
        real f2 =  (m_max.m_x - m_min.m_x) * (m_max.m_z - m_min.m_z);
        real f3 =  (m_max.m_y - m_min.m_y) * (m_max.m_z - m_min.m_z);
        return real(2.0) * f1 * f2 * f3;
    }

    /**
     * @brief  Combine 2 AABBs into 1 AABB.
     */
    inline void combine(const b3AABB& A, const b3AABB& B) {

        m_min = b3_min_coeff(A.m_min, B.m_min);
        m_max = b3_max_coeff(A.m_max, B.m_max);
    }

    /**
     * @brief Check this AABB contains another AABB.
     */
    inline bool contains(const b3AABB& other) const {

        bool result = true;

        result = result && m_min.m_x <= other.m_min.m_x;
        result = result && m_min.m_y <= other.m_min.m_y;
        result = result && m_min.m_z <= other.m_min.m_z;

        result = result && m_max.m_x >= other.m_max.m_x;
        result = result && m_max.m_y >= other.m_max.m_y;
        result = result && m_max.m_z >= other.m_max.m_z;

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
