
#ifndef BOX3D_B3_INERTIA_HPP
#define BOX3D_B3_INERTIA_HPP


#include "common/b3_types.hpp"
#include "dynamics/b3_transform.hpp"


struct b3MassProperty {

    real m_volume;

    real m_mass;

    /**
     * @brief The center of mass of the rigid body.
     * @note The m_center is just the integral part divide the volume
     */
    b3Vector3r m_center;

    b3Matrix3r m_Inertia;

};


#endif //BOX3D_B3_INERTIA_HPP
