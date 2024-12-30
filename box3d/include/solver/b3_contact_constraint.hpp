
#ifndef BOX3D_B3_CONTACT_CONSTRAINT_HPP
#define BOX3D_B3_CONTACT_CONSTRAINT_HPP


#include "common/b3_types.hpp"
#include "math/b3_mat33.hpp"
#include "math/b3_mat1212.hpp"
#include "collision/b3_collision.hpp"


struct b3BodySim;
class b3Contact;

struct b3VelocityConstraintPoint {
    b3Vec3r m_ra;
    b3Vec3r m_rb;
    real m_normal_impulse;
    real m_normal_mass;
    real m_tanget_mass;

    // bias velocity while preparing the velocity constraint.
    real m_bias_velocity;

    real m_iter_bias_velocity;

    real m_rhs_penetration;
    real m_relative_velocity = 0.0f;

    real m_wait = true;
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


    b3Vec3r m_va;
    b3Vec3r m_wa;
    b3Quatr m_qa;
    b3Vec3r m_pa;

    b3Vec3r m_vb;
    b3Vec3r m_wb;
    b3Quatr m_qb;
    b3Vec3r m_pb;

    real m_radius_a;
    real m_radius_b;
    real m_restitution;
    real m_restitution_threshold;

    real m_friction;

    real** m_JWJT = nullptr;
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


struct b3ContactSim {
    b3Contact* contact;
    b3BodySim* body_sim_a;
    b3BodySim* body_sim_b;

    b3Vec3r v_a;
    b3Vec3r w_a;
    b3Vec3r p_a;
    b3Quatr q_a;

    b3Vec3r v_b;
    b3Vec3r w_b;
    b3Vec3r p_b;
    b3Quatr q_b;

    b3Mat33r inv_I_a;
    b3Mat33r inv_I_b;

    real radius_a;
    real radius_b;

    b3Vec3r normal;

    real m_a;
    real m_b;
    real inv_m_a;
    real inv_m_b;

    real restitution;


    b3WorldManifold world_manifold;
    b3VelocityConstraintPoint points[8];
    int32 point_count;
};


#endif // BOX3D_B3_CONTACT_CONSTRAINT_HPP
