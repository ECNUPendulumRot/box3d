
#ifndef BOX3D_B3_CONTACT_CONSTRAINT_HPP
#define BOX3D_B3_CONTACT_CONSTRAINT_HPP

#include <common/b3_types.hpp>


struct b3VelocityConstraintPoint {
    b3Vector3r m_ra;
    b3Vector3r m_rb;
    real m_normal_collision_impulse = 0;
    real m_normal_contact_impulse = 0;
    real m_tangent_impulse = 0;
    real m_normal_mass = 0;
    real m_tanget_mass = 0;
    // double m_velocity_bias = 0;
    real m_rhs_restitution_velocity = 0;
    real m_rhs_penetration = 0;
};


struct b3ContactVelocityConstraint {
    b3VelocityConstraintPoint m_points[4];
    b3Vector3r m_normal;
    int32 m_index_a;
    int32 m_index_b;
    real m_inv_mass_a;
    real m_inv_mass_b;
    real m_mass_a;
    real m_mass_b;
    b3Matrix3r m_inv_I_a;
    b3Matrix3r m_inv_I_b;

    int32 m_point_count;
    int32 m_contact_index;

    real m_restitution;

    real m_penetration;
    // TODO
    real m_normal_collision_impulse = 0;
    real m_normal_contact_impulse = 0;

    b3Vector3r m_ra;
    b3Vector3r m_rb;

    real m_friction;

    // b3Vector3r m_world_center_a;
    // b3Vector3r m_world_center_b;


	// b2Mat22 normalMass;
	// b2Mat22 K;

	// float friction;
	// float threshold;
	// float tangentSpeed;
};

#endif // BOX3D_B3_CONTACT_CONSTRAINT_HPP
