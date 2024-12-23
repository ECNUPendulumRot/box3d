
#ifndef B3_MANIFOLD_POINT_HPP
#define B3_MANIFOLD_POINT_HPP

#include "math/b3_vec3.hpp"

class b3PersistentManifoldPoint {
public:

    b3PersistentManifoldPoint() = default;
    b3PersistentManifoldPoint(const b3Vec3r& pointA, const b3Vec3r& pointB,
                    const b3Vec3r& normal, real distance) :
                    m_local_pointA(pointA),
                    m_local_pointB(pointB),
                    m_normal_world_on_A(-normal),
                    m_normal_world_on_B(normal),
                    m_distance(distance)
    {
    }

    b3Vec3r m_local_pointA;
    b3Vec3r m_local_pointB;

    // TODO: delete
    b3Vec3r m_position_world_on_A;
    b3Vec3r m_position_world_on_B;

    // in Box3d, the normal is always pointing from A to B, in the world frame
    b3Vec3r m_normal_world_on_A;
    b3Vec3r m_normal_world_on_B;

    real m_distance = 0;
    real m_friction = 0;
    real m_restitution = 0;
    real m_rolling_friction = 0;
    real m_spinning_friction = 0;

    real m_applied_impulse = 0;
    real m_prevRHS = 0;
    real m_applied_tangent_impulse1 = 0;
    real m_applied_tangent_impulse2 = 0;

    // CFM ERP
    int m_lifetime = 0; // lifetime of the contact point in frames

    b3Vec3r m_lateral_friction_dir1;
    b3Vec3r m_lateral_friction_dir2;

    real get_distance() const {
        return m_distance;
    }

    int get_lifetime() const {
        return m_lifetime;
    }

    void set_distance(real distance) {
        m_distance = distance;
    }

    ///this returns the most recent applied impulse, to satisfy contact constraints by the constraint solver
    real get_applied_impulse() const
    {
        return m_applied_impulse;
    }
};


#endif