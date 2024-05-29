
#ifndef BOX3D_B3_CONTACT_CONSTRAINT_HPP
#define BOX3D_B3_CONTACT_CONSTRAINT_HPP


#include "common/b3_types.hpp"
#include "math/b3_mat33.hpp"
#include "math/b3_mat1212.hpp"
#include "collision/b3_collision.hpp"

struct b3VelocityConstraintPoint {

    b3Vec3r m_ra;
    b3Vec3r m_rb;
    real m_normal_impulse = 0;
    real m_normal_mass = 0;
    real m_tanget_mass = 0;
    real m_bias_velocity = 0;
    real m_rhs_penetration = 0;
    real m_relative_velocity = 0;
    bool wait = true;//this value is for situation 4
};


struct b3ContactVelocityConstraint {

    b3VelocityConstraintPoint m_points[8];
    b3Vec3r m_normal;
    int32 m_index_a;
    int32 m_index_b;
    real m_inv_mass_a;
    real m_inv_mass_b;

    b3Mat33r m_inv_I_a;
    b3Mat33r m_inv_I_b;

    int32 m_point_count;
    int32 m_contact_index;

    real m_restitution;
    real m_restitution_threshold;
    real m_penetration;
    // TODO
    real m_normal_collision_impulse = 0;
    real m_normal_contact_impulse = 0;

    real m_friction;

    real** m_JWJT = nullptr;

	// b2Mat22 normalMass;

	// float friction;
	// float threshold;
	// float tangentSpeed;
};


struct b3FrictionConstraint {
    int m_index_a;
    int m_index_b;
    real m_mass_a;
    real m_mass_b;
    real m_inv_mass_a;
    real m_inv_mass_b;
    b3Mat33r m_I_a;
    b3Mat33r m_I_b;
    b3Mat33r m_inv_I_a;
    b3Mat33r m_inv_I_b;
    b3Vec3r m_normal;
    int m_point_count;
    b3Vec3r m_ra;
    b3Vec3r m_rb;
    b3Vec3r m_ras[4];
    b3Vec3r m_rbs[4];

    real m_not_applied_support_impulse;
    // The tangent plane we divide to 2-axies.
    real m_max_friction_impulse;
    real m_friction_impulse[2];
    b3Vec3r m_friction_axis[2];
    real m_friction;
};


struct b3ContactPositionConstraint
{
    b3Vec3r m_local_points[8];
    b3Vec3r m_local_normal;
    b3Vec3r m_local_point;
    int32 m_index_a, m_index_b;
    real m_inv_mass_a, m_inv_mass_b;
    b3Vec3r m_center_a, m_center_b;
    b3Vec3r m_local_center_a, m_local_center_b;
    b3Mat33r m_inv_I_a, m_inv_I_b;

    real m_radius_a, m_radius_b;
    int32 m_point_count;

    b3Manifold::Type m_type;
};


#endif // BOX3D_B3_CONTACT_CONSTRAINT_HPP
