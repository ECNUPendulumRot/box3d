
#ifndef BOX3D_B3_INERTIA_HPP
#define BOX3D_B3_INERTIA_HPP


#include "common/b3_types.hpp"
#include "dynamics/b3_transform.hpp"


struct b3MassProperty {

    double m_volume;

    double m_mass;

    /**
     * @brief The center of mass of the rigid body.
     * @note The m_center is just the integral part divide the volume
     */
    b3Vector3d m_center;

    b3Matrix3d m_Inertia;

};


#endif //BOX3D_B3_INERTIA_HPP
