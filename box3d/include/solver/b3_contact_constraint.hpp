
#ifndef BOX3D_B3_CONTACT_CONSTRAINT_HPP
#define BOX3D_B3_CONTACT_CONSTRAINT_HPP

#include <common/b3_types.hpp>

namespace box3d {
    
    struct b3VelocityConstraintPoint;

    struct b3ContactVelocityConstraint;
}


struct box3d::b3VelocityConstraintPoint {
    b3Vector3d m_ra;
    b3Vector3d m_rb;
    double m_normal_impulse = 0;
    double m_tangent_impulse = 0;
    double m_normal_mass = 0;
    double m_tanget_mass = 0;
    // double m_velocity_bias = 0;
    double m_rhs_restitution_velocity = 0;
    double m_rhs_penetration = 0;
};


struct box3d::b3ContactVelocityConstraint {
    b3VelocityConstraintPoint m_points[4];
    b3Vector3d m_normal;
    int32 m_index_a;
    int32 m_index_b;
    double m_inv_mass_a;
    double m_inv_mass_b;
    b3Matrix3d m_inv_I_a;
    b3Matrix3d m_inv_I_b;

    int32 m_point_count;
    int32 m_contact_index;

    double m_restitution;

    // b3Vector3d m_world_center_a;
    // b3Vector3d m_world_center_b;


	// b2Mat22 normalMass;
	// b2Mat22 K;

	// float friction;
	// float threshold;
	// float tangentSpeed;
};

#endif // BOX3D_B3_CONTACT_CONSTRAINT_HPP
