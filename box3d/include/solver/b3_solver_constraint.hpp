
#ifndef B3_SOLVER_CONSTRAINT_HPP
#define B3_SOLVER_CONSTRAINT_HPP

class b3Body;

#include "b3_solver_body.hpp"

struct b3SolverConstraint {

    b3Vec3r m_rel_pos1_cross_normal;
    b3Vec3r m_contact_normal_1;

    b3Vec3r m_rel_pos2_cross_normal;
    b3Vec3r m_contact_normal_2;

    b3Vec3r m_angular_componentA;
    b3Vec3r m_angular_componentB;

    real m_applied_push_impulse;
    real m_applied_impulse;

    real m_friction;
    real m_jac_diag_AB_inv;
    real m_rhs;
    real m_cfm;

    real m_lower_limit;
    real m_upper_limit;
    real m_rhs_penetration;

    void* m_original_contact_point;

    int m_override_num_solver_iterations;
    int m_friction_index;
    int m_solver_body_idA;
    int m_solver_body_idB;
};

#endif