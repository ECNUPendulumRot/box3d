
#ifndef BOX3D_B3_CONTACT_CONSTRAINT_HPP
#define BOX3D_B3_CONTACT_CONSTRAINT_HPP

#include <common/b3_types.hpp>
#include "dynamics/b3_body_def.hpp"

struct b3VelocityConstraintPoint {
    b3Vector3r m_ra;
    b3Vector3r m_rb;

    real m_normal_mass = 0;
    real m_tanget_mass = 0;

    Eigen::Vector<double, 12> Ja = Eigen::Vector<double, 12>::Zero();
    Eigen::Vector<double, 12> Jb = Eigen::Vector<double, 12>::Zero();
};


struct b3AffineContactVelocityConstraint {

    b3VelocityConstraintPoint m_points[4];

    int32 m_index_a;
    int32 m_index_b;
    real m_inv_mass_a;
    real m_inv_mass_b;
    real m_mass_a;
    real m_mass_b;

    Eigen::Vector3<real> m_normal;
    Eigen::Matrix<real, 12, 12> m_affine_I_a;
    Eigen::Matrix<real, 12, 12> m_affine_I_b;
    Eigen::Matrix<real, 12, 12> m_affine_inv_I_a;
    Eigen::Matrix<real, 12, 12> m_affine_inv_I_b;

    int32 m_point_count;
    int32 m_contact_index;

    b3BodyType m_type_a;
    b3BodyType m_type_b;

    real m_restitution;

    real m_penetration;
    // TODO

    void flip() {
        std::swap(m_index_a, m_index_b);
        std::swap(m_inv_mass_a, m_inv_mass_b);
        std::swap(m_mass_a, m_mass_b);
        std::swap(m_affine_I_a, m_affine_I_b);
        std::swap(m_affine_inv_I_a, m_affine_inv_I_b);
        std::swap(m_type_a, m_type_b);
    }
};


struct b3FrictionConstraint {
    int m_index_a;
    int m_index_b;
    real m_mass_a;
    real m_mass_b;
    real m_inv_mass_a;
    real m_inv_mass_b;
    b3Matrix3r m_I_a;
    b3Matrix3r m_I_b;
    b3Matrix3r m_inv_I_a;
    b3Matrix3r m_inv_I_b;
    b3Vector3r m_normal;
    int m_point_count;
    b3Vector3r m_ra;
    b3Vector3r m_rb;
    b3Vector3r m_ras[4];
    b3Vector3r m_rbs[4];

    real m_not_applied_support_impulse;
    // The tangent plane we divide to 2-axies.
    real m_max_friction_impulse;
    real m_friction_impulse[2];
    b3Vector3r m_friction_axis[2];
    real m_friction;
};

#endif // BOX3D_B3_CONTACT_CONSTRAINT_HPP
