
#ifndef BOX3D_B3_CONTACT_CONSTRAINT_HPP
#define BOX3D_B3_CONTACT_CONSTRAINT_HPP

#include <common/b3_types.hpp>


struct b3VelocityConstraintPoint {
    b3Vec3r m_ra;
    b3Vec3r m_rb;
    real m_normal_collision_impulse = 0;
    real m_normal_contact_impulse = 0;
    real m_tangent1_impulse = 0;
    real m_tangent2_impulse = 0;
    real m_normal_mass = 0;
    real m_tangent1_mass = 0;
    real m_tangent2_mass = 0;
    // double m_velocity_bias = 0;
    real m_rhs_restitution_velocity = 0;
    real m_rhs_penetration = 0;
};

struct b3ContactVelocityConstraint {
    b3VelocityConstraintPoint m_points[8];
    b3Vec3r m_normal;
    b3Vec3r m_tangent1, m_tangent2;
    int32 m_index_a;
    int32 m_index_b;
    real m_inv_mass_a;
    real m_inv_mass_b;
    real m_mass_a;
    real m_mass_b;
    b3Mat33r m_I_a;
    b3Mat33r m_I_b;
    b3Mat33r m_inv_I_a;
    b3Mat33r m_inv_I_b;

    real m_target_normal_angular_velocity;

    int32 m_point_count;
    int32 m_contact_index;

    b3Mat33r m_inv_I_ab;

    real m_restitution;

    real m_penetration;
    // TODO
    real m_normal_collision_impulse = 0;
    real m_normal_contact_impulse = 0;

    real m_friction;

    bool m_is_sphere[2];
    real m_radius[2];
    real m_inv_radius[2];
    real m_use_support_lambda[2][2];
};


#endif // BOX3D_B3_CONTACT_CONSTRAINT_HPP
