//
// Created by sherman on 11/8/23.
//

#ifndef BOX3D_B3_AABB_HPP
#define BOX3D_B3_AABB_HPP

#include "common/b3_types.hpp"


class b3AABB {

    friend class b3Mesh;
    friend class b3CubeShape;
    friend class b3SphereShape;
    friend class b3DynamicTree;

    /**
     * The lower bound of the AABB.
     */
    b3Vector3d m_min;

    /**
     * The upper bound of the AABB.
     */
    b3Vector3d m_max;

public:

    b3AABB() = default;

    b3AABB(const b3Vector3d& lower_bound, const b3Vector3d& upper_bound);

    b3AABB(const Eigen::Vector3d& lower_bound, const Eigen::Vector3d& upper_bound);

    /**
     * @brief Get the center coordinate of the AABB.
     * @return The center coordinate of the AABB.
     */
    inline b3Vector3d center() const {

        return 0.5 * (m_max - m_min);
    }

    /**
     * @brief Get the surface area of the AABB.
     * @return The surface area of the AABB.
     */
    inline double get_surface_area(){

        double f1 =  (m_max.x() - m_min.x()) * (m_max.y() - m_min.y());
        double f2 =  (m_max.x() - m_min.x()) * (m_max.z() - m_min.z());
        double f3 =  (m_max.y() - m_min.y()) * (m_max.z() - m_min.z());
        return 2.0 * f1 * f2 * f3;
    }

    /**
     * @brief  Combine 2 AABBs into 1 AABB.
     */
    inline void combine(const b3AABB& A, const b3AABB& B) {

        m_min = b3_min_coeff(A.m_min, B.m_min);
        m_max = b3_max_coeff(A.m_max, B.m_max);
    }

    inline bool contains(const b3AABB& other) const {

        bool result = true;

        result = result && m_min.x() <= other.m_min.x();
        result = result && m_min.y() <= other.m_min.y();
        result = result && m_min.z() <= other.m_min.z();

        result = result && m_max.x() >= other.m_max.x();
        result = result && m_max.y() >= other.m_max.y();
        result = result && m_max.z() >= other.m_max.z();

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
