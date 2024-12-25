
#include "dynamics/constraint/b3_point_point_constraint.hpp"
#include "solver/b3_solver_constraint.hpp"

#include "spdlog/spdlog.h"

// bodyA is static body and bodyB is dynamic body
b3Point2PointConstraint::b3Point2PointConstraint(b3Body* bodyA, b3Body* bodyB, const b3Vec3r& pivot_inA, const b3Vec3r& pivot_inB, bool collide_connected) :
    b3ConstraintBase(bodyA, bodyB, collide_connected),
    m_pivot_inA(pivot_inA),
    m_pivot_inB(pivot_inB),
    m_use_solve_constraint_obsolete(false)
{
    m_pivotA_world = m_bodyA->get_world_transform().transform(m_pivot_inA);
}

void b3Point2PointConstraint::apply_constraint_force_and_torque(b3SolverBody& solver_bodyB) {
    real force = -(solver_bodyB.m_external_force_impulse.dot(m_apply_force_direction));
    if (force > 0.f) {
        b3Vec3r apply_force = force * m_apply_force_direction;
        b3Vec3r r = m_pivotA_world - solver_bodyB.m_world_transform.position();

        solver_bodyB.m_external_force_impulse.set_zero();
        // solver_bodyB.m_external_force_impulse += apply_force * m_apply_force_direction;
        solver_bodyB.m_external_torque_impulse += r.cross(apply_force);
    }
}

void b3Point2PointConstraint::build_jacobian() {
    m_applied_impulse = 0.f;

    b3Vec3r normal(0, 0, 0);
    for (int i = 0; i < 3; i++) {
        normal[i] = 1;
        new (&m_jac[i]) b3JacobianEntry(
            m_bodyA->get_world_transform().rotation_matrix().transpose(),
            m_bodyB->get_world_transform().rotation_matrix().transpose(),
            m_bodyA->get_world_transform().transform(m_pivot_inA) - m_bodyA->get_world_transform().position(),
            m_bodyB->get_world_transform().transform(m_pivot_inB) - m_bodyB->get_world_transform().position(),
            normal,
            m_bodyA->get_inv_inertia_local(),
            m_bodyA->get_inv_mass(),
            m_bodyB->get_inv_inertia_local(),
            m_bodyB->get_inv_mass()
            );
        normal[i] = 0;
    }
}

void b3Point2PointConstraint::get_info1(b3ConstraintBase::b3ConstraintInfo1* info) {
    get_info1_non_virtual(info);
}

void b3Point2PointConstraint::get_info1_non_virtual(b3ConstraintBase::b3ConstraintInfo1* info) {
    if (m_use_solve_constraint_obsolete) {
        info->m_num_constraint_rows = 0;
        info->nub = 0;
    } else {
        info->m_num_constraint_rows = 3;
        info->nub = 3;
    }
}

void b3Point2PointConstraint::get_info2(b3ConstraintBase::b3ConstraintInfo2 *info) {
    get_info2_non_virtual(info, m_bodyA->get_world_transform(), m_bodyB->get_world_transform());
}

void b3Point2PointConstraint::get_info2_non_virtual(b3ConstraintBase::b3ConstraintInfo2 *info,
    const b3Transformr &body0_trans, const b3Transformr &body1_trans) {

    b3_assert(!m_use_solve_constraint_obsolete);

    //retrieve matrices

    // anchor points in global coordinates with respect to body PORs.

    // set jacobian
    info->m_solver_constraint[0].m_contact_normal_1[0] = 1;
    info->m_solver_constraint[1].m_contact_normal_1[1] = 1;
    info->m_solver_constraint[2].m_contact_normal_1[2] = 1;

    b3Vec3r a1 = body0_trans.rotation_matrix() * get_pivot_inA();
    {
        b3Vec3r* angular0 = &info->m_solver_constraint[0].m_rel_pos1_cross_normal;
        b3Vec3r* angular1 = &info->m_solver_constraint[1].m_rel_pos1_cross_normal;
        b3Vec3r* angular2 = &info->m_solver_constraint[2].m_rel_pos1_cross_normal;
        angular0->set(0, a1[2], -a1[1]);
        angular1->set(-a1[2], 0, a1[0]);
        angular2->set(a1[1], -a1[0], 0);
    }

    info->m_solver_constraint[0].m_contact_normal_2[0] = -1;
    info->m_solver_constraint[1].m_contact_normal_2[1] = -1;
    info->m_solver_constraint[2].m_contact_normal_2[2] = -1;

    b3Vec3r a2 = body1_trans.rotation_matrix() * get_pivot_inB();

    {
        //	btVector3 a2n = -a2;
        b3Vec3r* angular0 = &info->m_solver_constraint[0].m_rel_pos2_cross_normal;
        b3Vec3r* angular1 = &info->m_solver_constraint[1].m_rel_pos2_cross_normal;
        b3Vec3r* angular2 = &info->m_solver_constraint[2].m_rel_pos2_cross_normal;
        angular0->set(0, -a2[2], a2[1]);
        angular1->set(a2[2], 0, -a2[0]);
        angular2->set(-a2[1], a2[0], 0);
    }

    // set right hand side
    real currERP = info->erp;
    real k = info->fps * currERP;

    for (int j = 0; j < 3; j++) {
        info->m_solver_constraint[j].m_rhs = k * (a2[j] + body1_trans.position()[j] - a1[j] - body0_trans.position()[j]);
    }
    info->m_damping = 1;
}