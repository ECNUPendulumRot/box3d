
#include "solver/b3_solver_lxj.hpp"

#include "dynamics/b3_island.hpp"
#include "dynamics/b3_body.hpp"

#include "collision/b3_contact.hpp"
#include "collision/b3_persistent_manifold.hpp"

#include "common/b3_time_step.hpp"

#include <iostream>

static real resolve_single_constraint_row_generic(b3SolverBody& bodyA, b3SolverBody& bodyB, b3SolverConstraint& c) {
    real delta_impulse = c.m_rhs - c.m_applied_impulse * c.m_cfm;
    const real delta_velA_dotn = c.m_contact_normal_1.dot(bodyA.internal_get_delta_linear_velocity()) + c.m_rel_pos1_cross_normal.dot(bodyA.internal_get_delta_angular_velocity());
    const real delta_velB_dotn = c.m_contact_normal_2.dot(bodyB.internal_get_delta_linear_velocity()) + c.m_rel_pos2_cross_normal.dot(bodyB.internal_get_delta_angular_velocity());

    delta_impulse -= (delta_velA_dotn + delta_velB_dotn) * c.m_jac_diag_AB_inv;

    const real sum = c.m_applied_impulse + delta_impulse;
    if (sum < c.m_lower_limit) {
        delta_impulse = c.m_lower_limit - c.m_applied_impulse;
        c.m_applied_impulse = c.m_lower_limit;
    } else if (sum > c.m_upper_limit) {
        delta_impulse = c.m_upper_limit - c.m_applied_impulse;
        c.m_applied_impulse = c.m_upper_limit;
    } else {
        c.m_applied_impulse = sum;
    }
    bodyA.internal_apply_impulse(c.m_contact_normal_1 * bodyA.internal_get_inv_mass(), c.m_angular_componentA, delta_impulse);
    bodyB.internal_apply_impulse(c.m_contact_normal_2 * bodyB.internal_get_inv_mass(), c.m_angular_componentB, delta_impulse);

    return delta_impulse / c.m_jac_diag_AB_inv;
}

static real resolve_single_constraint_row_lower_limit(b3SolverBody& bodyA, b3SolverBody& bodyB, b3SolverConstraint& c) {
    real delta_impulse = c.m_rhs - c.m_applied_impulse * c.m_cfm;
    const real delta_velA_dotn = c.m_contact_normal_1.dot(bodyA.internal_get_delta_linear_velocity()) + c.m_rel_pos1_cross_normal.dot(bodyA.internal_get_delta_angular_velocity());
    const real delta_velB_dotn = c.m_contact_normal_2.dot(bodyB.internal_get_delta_linear_velocity()) + c.m_rel_pos2_cross_normal.dot(bodyB.internal_get_delta_angular_velocity());

    delta_impulse -= (delta_velA_dotn + delta_velB_dotn) * c.m_jac_diag_AB_inv;

    const real sum = c.m_applied_impulse + delta_impulse;
    if (sum < c.m_lower_limit) {
        delta_impulse = c.m_lower_limit - c.m_applied_impulse;
        c.m_applied_impulse = c.m_lower_limit;
    } else {
        c.m_applied_impulse = sum;
    }

    bodyA.internal_apply_impulse(c.m_contact_normal_1 * bodyA.internal_get_inv_mass(), c.m_angular_componentA, delta_impulse);
    bodyB.internal_apply_impulse(c.m_contact_normal_2 * bodyB.internal_get_inv_mass(), c.m_angular_componentB, delta_impulse);

    return delta_impulse / c.m_jac_diag_AB_inv;
}

real b3SolverLxj::restitution_curve(real rel_vel, real restitution, real velocity_threshold) {
    if (b3_abs(rel_vel) < velocity_threshold) {
        return 0.;
    }
    return restitution * -rel_vel;
}

void b3SolverLxj::solve_group(b3Island* island, b3TimeStep* time_step) {

    if (time_step->m_dt != 0.f) {
        solve_group_setup(island, time_step);

        solve_group_iterations(island, time_step);

        solve_group_finish(time_step);
    }
}


void b3SolverLxj::solve_group_iterations(b3Island* island, b3TimeStep* time_step) {
    solve_group_positions(time_step);
    for (int i = 0; i < time_step->m_iterations; i++) {
        m_least_squares_residual = solver_single_iteration(island, time_step);

        if (m_least_squares_residual <= time_step->m_least_squares_residual_threshold) {
            break;
        }
    }
}


void b3SolverLxj::solve_group_positions(b3TimeStep* time_step) {
    for (int i = 0; i < time_step->m_position_iterations; i++) {
        real least_squares_residual = 0.f;
        for (int j = 0; j < m_tmp_solver_contact_constraint_pool.size(); j++) {
            b3SolverConstraint& solve_constraint = m_tmp_solver_contact_constraint_pool[j];

            real residual = resolve_penetration_impulse(m_tmp_solver_bodies_pool[solve_constraint.m_solver_body_idA],
                                                       m_tmp_solver_bodies_pool[solve_constraint.m_solver_body_idB],
                                                       solve_constraint);
            least_squares_residual = b3_max(least_squares_residual, residual * residual);
        }
        if (least_squares_residual <= time_step->m_least_squares_residual_threshold) {
            break;
        }
    }
}


real b3SolverLxj::resolve_penetration_impulse(b3SolverBody &bodyA, b3SolverBody &bodyB, b3SolverConstraint &c) {
    real delta_impulse = 0.f;

    if (c.m_rhs_penetration > 0.f) {
        delta_impulse = c.m_rhs_penetration - c.m_applied_push_impulse * c.m_cfm;
        const real delta_velA_dotn = c.m_contact_normal_1.dot(bodyA.internal_get_push_velocity()) + c.m_rel_pos1_cross_normal.dot(bodyA.internal_get_turn_velocity());
        const real delta_velB_dotn = c.m_contact_normal_2.dot(bodyB.internal_get_push_velocity()) + c.m_rel_pos2_cross_normal.dot(bodyB.internal_get_turn_velocity());

        delta_impulse -= (delta_velA_dotn + delta_velB_dotn) * c.m_jac_diag_AB_inv;
        const real sum = c.m_applied_push_impulse + delta_impulse;
        if (sum < c.m_lower_limit) {
            delta_impulse = c.m_lower_limit - c.m_applied_push_impulse;
            c.m_applied_push_impulse = c.m_lower_limit;
        } else {
            c.m_applied_push_impulse = sum;
        }

        bodyA.internal_apply_push_impulse(c.m_contact_normal_1 * bodyA.internal_get_inv_mass(), c.m_angular_componentA, delta_impulse);
        bodyB.internal_apply_push_impulse(c.m_contact_normal_2 * bodyB.internal_get_inv_mass(), c.m_angular_componentB, delta_impulse);
    }

    return delta_impulse / c.m_jac_diag_AB_inv;
}


real b3SolverLxj::solver_single_iteration(b3Island* island, b3TimeStep* time_step) {
    real least_squares_residual = 0.0;

    /// solve all joint constraints
    for (int j = 0; j < m_tmp_solver_non_contact_constraint_pool.size(); j++) {
        b3SolverConstraint& constraint = m_tmp_solver_non_contact_constraint_pool[j];
        real residual = resolve_single_constraint_row_generic(m_tmp_solver_bodies_pool[constraint.m_solver_body_idA],
                                                              m_tmp_solver_bodies_pool[constraint.m_solver_body_idB],
                                                              constraint);
        least_squares_residual = b3_max(least_squares_residual, residual * residual);
    }

    for (int j = 0; j < island->get_constraint_count(); j++) {
        b3ConstraintBase* constraint = island->get_constraint(j);
        if (constraint->is_enabled()) {
            int bodyA_id = get_or_init_solver_body(*constraint->get_bodyA(), time_step->m_dt);
            int bodyB_id = get_or_init_solver_body(*constraint->get_bodyB(), time_step->m_dt);
            b3SolverBody& bodyA = m_tmp_solver_bodies_pool[bodyA_id];
            b3SolverBody& bodyB = m_tmp_solver_bodies_pool[bodyB_id];
            constraint->solve_constraint_obsolete(bodyA, bodyB, time_step->m_dt);
        }
    }

    // solve the friction constraints after all contact constraints
    // std::cout << "solve contact constraints" << std::endl;
    for (int j = 0; j < m_tmp_solver_contact_constraint_pool.size(); j++) {
        b3SolverConstraint& solve_manifold = m_tmp_solver_contact_constraint_pool[j];
        real residual = resolve_single_constraint_row_lower_limit(m_tmp_solver_bodies_pool[solve_manifold.m_solver_body_idA],
                                                                   m_tmp_solver_bodies_pool[solve_manifold.m_solver_body_idB],
                                                                   solve_manifold);
        least_squares_residual = b3_max(least_squares_residual, residual * residual);
    }
    // std::cout << "solve contact friction constraints" << std::endl;
    for (int j = 0; j < m_tmp_solver_contact_friction_constraint_pool.size(); j++) {
        b3SolverConstraint& solve_manifold = m_tmp_solver_contact_friction_constraint_pool[j];
        real total_impulse = m_tmp_solver_contact_constraint_pool[solve_manifold.m_friction_index].m_applied_impulse;

        if (total_impulse > real(0)) {
            solve_manifold.m_lower_limit = -(solve_manifold.m_friction * total_impulse);
            solve_manifold.m_upper_limit = solve_manifold.m_friction * total_impulse;

            real residual = resolve_single_constraint_row_generic(m_tmp_solver_bodies_pool[solve_manifold.m_solver_body_idA],
                                                                  m_tmp_solver_bodies_pool[solve_manifold.m_solver_body_idB],
                                                                  solve_manifold);
            least_squares_residual = b3_max(least_squares_residual, residual * residual);
        }
    }
    // std::cout << "solve contact spinning and rolling friction constraints" << std::endl;
    for (int j = 0; j < m_tmp_solver_rolling_friction_constraint_pool.size(); j++) {
        b3SolverConstraint& rolling_friction_constraint = m_tmp_solver_rolling_friction_constraint_pool[j];
        real total_impulse = m_tmp_solver_contact_constraint_pool[rolling_friction_constraint.m_friction_index].m_applied_impulse;

        if (total_impulse > real(0)) {
            real rolling_friction_magnitude = rolling_friction_constraint.m_friction * total_impulse;
            if (rolling_friction_magnitude > rolling_friction_constraint.m_friction) {
                rolling_friction_magnitude = rolling_friction_constraint.m_friction;
            }
            rolling_friction_constraint.m_lower_limit = -rolling_friction_magnitude;
            rolling_friction_constraint.m_upper_limit = rolling_friction_magnitude;

            real residual = resolve_single_constraint_row_generic(m_tmp_solver_bodies_pool[rolling_friction_constraint.m_solver_body_idA],
                                                                   m_tmp_solver_bodies_pool[rolling_friction_constraint.m_solver_body_idB],
                                                                   rolling_friction_constraint);
            least_squares_residual = b3_max(least_squares_residual, residual * residual);
        }
    }
    return least_squares_residual;
}


void b3SolverLxj::solve_group_finish(b3TimeStep* time_step) {
    if (time_step->m_warm_starting) {
        write_back_contacts(time_step);
    }

    write_back_bodies(time_step->m_dt);

    m_tmp_solver_bodies_pool.clear();
    m_tmp_solver_contact_constraint_pool.clear();
    m_tmp_solver_contact_friction_constraint_pool.clear();
    m_tmp_solver_rolling_friction_constraint_pool.clear();
}


void b3SolverLxj::write_back_contacts(b3TimeStep* time_step) {
    for (int j = 0; j < m_tmp_solver_contact_constraint_pool.size(); j++) {
        const b3SolverConstraint &solve_manifold = m_tmp_solver_contact_constraint_pool[j];
        auto *pt = (b3PersistentManifoldPoint *) solve_manifold.m_original_contact_point;
        b3_assert(pt);
        pt->m_applied_impulse = solve_manifold.m_applied_impulse;
    }
    if (time_step->m_solve_use_2_friction_directions) {
        for (int j = 0; j < m_tmp_solver_contact_friction_constraint_pool.size(); j = j + 2) {
            auto* pt = (b3PersistentManifoldPoint*)m_tmp_solver_contact_friction_constraint_pool[j].m_original_contact_point;
            b3_assert(pt);
            pt->m_applied_tangent_impulse1 = m_tmp_solver_contact_friction_constraint_pool[j].m_applied_impulse;
            pt->m_applied_tangent_impulse2 = m_tmp_solver_contact_friction_constraint_pool[j + 1].m_applied_impulse;
        }
    } else {
        for (int j = 0; j < m_tmp_solver_contact_friction_constraint_pool.size(); j++) {
            auto* pt = (b3PersistentManifoldPoint*)m_tmp_solver_contact_friction_constraint_pool[j].m_original_contact_point;
            b3_assert(pt);
            pt->m_applied_tangent_impulse1 = m_tmp_solver_contact_friction_constraint_pool[j].m_applied_impulse;
        }
    }
}


void b3SolverLxj::write_back_bodies(real dt) {
    for (int j = 0; j < m_tmp_solver_bodies_pool.size(); j++) {
        b3Body* body = m_tmp_solver_bodies_pool[j].m_original_body;
        if (body) {
            m_tmp_solver_bodies_pool[j].write_back_velocity_and_transform(dt);

            body->set_linear_velocity(m_tmp_solver_bodies_pool[j].m_linear_velocity + m_tmp_solver_bodies_pool[j].m_external_force_impulse);
            body->set_angular_velocity(m_tmp_solver_bodies_pool[j].m_angular_velocity + m_tmp_solver_bodies_pool[j].m_external_torque_impulse);

            body->set_companion_id(-1);
        }
    }
}


void b3SolverLxj::solve_group_setup(b3Island *island, b3TimeStep *time_step) {
    m_fixed_body_id = -1;
    m_max_override_num_solver_iterations = 0;

    convert_bodies(island, time_step);
    convert_contacts(island, time_step);

    convert_joints(island, time_step);

    int num_constraint_pool = (int)m_tmp_solver_contact_constraint_pool.size();
    int num_friction_pool = (int)m_tmp_solver_contact_friction_constraint_pool.size();
    // TODO: 2 * num_constraint_pool == num_friction_pool ? Delete order vector ?
//    m_order_temp_constraint_pool.resize(num_constraint_pool * 2);
//    m_order_friction_constraint_pool.resize(num_friction_pool);
//    for (int i = 0; i < num_constraint_pool; i++) {
//        m_order_temp_constraint_pool[i] = i;
//    }
//    for (int i = 0; i < num_friction_pool; i++) {
//        m_order_friction_constraint_pool[i] = i;
//    }
}


void b3SolverLxj::convert_bodies(b3Island *island, b3TimeStep* time_step) {
    int body_count = island->get_body_count();
    b3Body** bodies = island->get_bodies();

    for (int i = 0; i < body_count; i++) {
        bodies[i]->set_companion_id(-1);
    }

    m_tmp_solver_bodies_pool.reserve(body_count + 1);
    m_tmp_solver_bodies_pool.resize(0);

    for (int i = 0; i < body_count; i++) {
        int body_id = get_or_init_solver_body(*bodies[i], time_step->m_dt);

        if (bodies[i]->get_inv_mass() > real(0.0)) {
            b3SolverBody& solver_body = m_tmp_solver_bodies_pool[body_id];
            // 计算科里奥利力矩
            b3Vec3r gyro_torque =  bodies[i]->compute_gyro_scopic_implicit(time_step->m_dt);;
            solver_body.m_external_torque_impulse += gyro_torque;

        }
    }
}


void b3SolverLxj::convert_contacts(b3Island *island, b3TimeStep *time_step) {
    b3Contact** contacts = island->get_contacts();
    int contact_count = island->get_contacts_count();

    b3PersistentManifold* manifold = nullptr;
    for (int i = 0; i < contact_count; i++) {
        manifold = contacts[i]->get_persistent_manifold();
        convert_contact(manifold, time_step);
    }
}


void b3SolverLxj::convert_joints(b3Island* island, b3TimeStep* time_step) {

    b3ConstraintBase* c = island->get_constraint(0);
    b3Body* bodyA = c->get_bodyA();
    b3Body* bodyB = c->get_bodyB();

    int solver_body_idA = get_or_init_solver_body(*bodyA, time_step->m_dt);
    int solver_body_idB = get_or_init_solver_body(*bodyB, time_step->m_dt);
    b3SolverBody& solver_bodyA = m_tmp_solver_bodies_pool[solver_body_idA];
    b3SolverBody& solver_bodyB = m_tmp_solver_bodies_pool[solver_body_idB];


    if (solver_bodyA.m_original_body == nullptr) {
        b3Vec3r pivotA = bodyA->get_world_transform().transform(b3Vec3r(0, 0, -0.4));
        // bodyA is the fixed body
        b3Vec3r r = pivotA - solver_bodyB.m_world_transform.position();
        b3Vec3r external_force = -solver_bodyB.m_external_force_impulse;
        solver_bodyB.m_external_force_impulse += external_force;
        solver_bodyB.m_external_torque_impulse = r.cross(external_force);
    } else {
        b3Vec3r pivotB = bodyB->get_world_transform().transform(b3Vec3r(0, 0, -0.4));
        // bodyB is the fixed body
        b3Vec3r r = pivotB - solver_bodyA.m_world_transform.position();
        b3Vec3r external_force = -solver_bodyA.m_external_force_impulse;
        solver_bodyA.m_external_force_impulse += external_force;
        solver_bodyA.m_external_torque_impulse = r.cross(external_force);
    }
    // return;

    int32 num_constraints = island->get_constraint_count();
    for (int j = 0; j < num_constraints; j++) {
        b3ConstraintBase* constraint = island->get_constraint(j);
        constraint->build_jacobian();
        constraint->internal_set_applied_impulse(0.0f);
    }

    int total_num_rows = 0;
    m_tmp_constraint_sizes_pool.resize(num_constraints);
    // Calculate the total number of constraint rows
    for (int i = 0; i < num_constraints; i++) {
        b3ConstraintBase::b3ConstraintInfo1& info1 = m_tmp_constraint_sizes_pool[i];

        b3ConstraintBase* constraint = island->get_constraint(i);
        if (constraint->is_enabled()) {
            constraint->get_info1(&info1);
        } else {
            info1.m_num_constraint_rows = 0;
            info1.nub = 0;
        }
        total_num_rows += info1.m_num_constraint_rows;
    }
    m_tmp_solver_non_contact_constraint_pool.resize(total_num_rows);

    // set up the b3SolverConstraints
    int current_row = 0;
    for (int i = 0; i < num_constraints; i++) {
        const b3ConstraintBase::b3ConstraintInfo1& info1 = m_tmp_constraint_sizes_pool[i];

        if (info1.m_num_constraint_rows) {
            b3_assert(current_row < total_num_rows);

            b3SolverConstraint* current_constraint_row = &m_tmp_solver_non_contact_constraint_pool[current_row];
            b3ConstraintBase* constraint = island->get_constraint(i);
            b3Body* bodyA = constraint->get_bodyA();
            b3Body* bodyB = constraint->get_bodyB();

            int solver_body_idA = get_or_init_solver_body(*bodyA, time_step->m_dt);
            int solver_body_idB = get_or_init_solver_body(*bodyB, time_step->m_dt);

            convert_joint(current_constraint_row, constraint, info1, solver_body_idA, solver_body_idB, time_step);
        }
        current_row += info1.m_num_constraint_rows;
    }
}

void b3SolverLxj::convert_joint(b3SolverConstraint* current_constraint_row, b3ConstraintBase* constraint,
    const b3ConstraintBase::b3ConstraintInfo1& info1, int solver_body_idA, int solver_body_idB, b3TimeStep* time_step) {

    b3Body* bodyA = constraint->get_bodyA();
    b3Body* bodyB = constraint->get_bodyB();

    const b3SolverBody* bodyA_ptr = &m_tmp_solver_bodies_pool[solver_body_idA];
    const b3SolverBody* bodyB_ptr = &m_tmp_solver_bodies_pool[solver_body_idB];

    m_max_override_num_solver_iterations = time_step->m_iterations;

    for (int j = 0; j < info1.m_num_constraint_rows; j++) {
        current_constraint_row[j].m_lower_limit = -b3_real_max;
        current_constraint_row[j].m_upper_limit = b3_real_max;
        current_constraint_row[j].m_applied_impulse = 0.0f;
        current_constraint_row[j].m_applied_push_impulse = 0.0f;
        current_constraint_row[j].m_solver_body_idA = solver_body_idA;
        current_constraint_row[j].m_solver_body_idB = solver_body_idB;
        current_constraint_row[j].m_override_num_solver_iterations = m_max_override_num_solver_iterations;
    }

    b3ConstraintBase::b3ConstraintInfo2 info2;
    info2.fps = 1.f / time_step->m_dt;
    info2.erp = time_step->m_erp;
    info2.m_solver_constraint = current_constraint_row;
//    info2.m_J1linear_axis = current_constraint_row->m_contact_normal_1;
//    info2.m_J1angular_axis = current_constraint_row->m_rel_pos1_cross_normal;
//    info2.m_J2linear_axis = current_constraint_row->m_contact_normal_2;
//    info2.m_J2angular_axis = current_constraint_row->m_rel_pos2_cross_normal;
//    info2.rowskip = sizeof(b3SolverConstraint) / sizeof(real);
//
//    b3_assert(info2.rowskip * sizeof(real) == sizeof(b3SolverConstraint));
//    info2.m_constraint_error = &current_constraint_row->m_rhs;
    current_constraint_row->m_cfm = time_step->m_global_cfm;
    info2.m_num_iterations = time_step->m_iterations;
    // 初始化所有约束所需要的法线等信息
    constraint->get_info2(&info2);

    // finalize the constraint setup
    for (int j = 0; j < info1.m_num_constraint_rows; j++) {
        b3SolverConstraint& solver_constraint = current_constraint_row[j];

        solver_constraint.m_original_contact_point = constraint;

        const b3Vec3r& ftorque_axis1 = solver_constraint.m_rel_pos1_cross_normal;
        solver_constraint.m_angular_componentA = constraint->get_bodyA()->get_inv_inertia_tensor_world() * ftorque_axis1;

        const b3Vec3r& ftorque_axis2 = solver_constraint.m_rel_pos2_cross_normal;
        solver_constraint.m_angular_componentB = constraint->get_bodyB()->get_inv_inertia_tensor_world() * ftorque_axis2;

        b3Vec3r iMJlA = solver_constraint.m_contact_normal_1 * bodyA->get_inv_mass();
        b3Vec3r iMJaA = bodyA->get_inv_inertia_tensor_world() * solver_constraint.m_rel_pos1_cross_normal;
        b3Vec3r iMJlB = solver_constraint.m_contact_normal_2 * bodyB->get_inv_mass();
        b3Vec3r iMJaB = bodyB->get_inv_inertia_tensor_world() * solver_constraint.m_rel_pos2_cross_normal;

        // compute JM_invJ^T
        real sum = iMJlA.dot(solver_constraint.m_contact_normal_1);
        sum += iMJaA.dot(solver_constraint.m_rel_pos1_cross_normal);
        sum += iMJlB.dot(solver_constraint.m_contact_normal_2);
        sum += iMJaB.dot(solver_constraint.m_rel_pos2_cross_normal);
        real fsum = b3_abs(sum);
        solver_constraint.m_jac_diag_AB_inv = fsum > b3_real_epsilon ? 1.0 / sum : 0.f;
        // compute the external force and tourque impulse
        b3Vec3r external_force_impulseA = bodyA_ptr->m_original_body ? bodyA_ptr->m_external_force_impulse : b3Vec3r::zero();
        b3Vec3r external_torque_impulseA = bodyA_ptr->m_original_body ? bodyA_ptr->m_external_torque_impulse : b3Vec3r::zero();
        b3Vec3r external_force_impulseB = bodyB_ptr->m_original_body ? bodyB_ptr->m_external_force_impulse : b3Vec3r::zero();
        b3Vec3r external_torque_impulseB = bodyB_ptr->m_original_body ? bodyB_ptr->m_external_torque_impulse : b3Vec3r::zero();

        real vel1_dotn = solver_constraint.m_contact_normal_1.dot(bodyA->get_linear_velocity() + external_force_impulseA) +
            solver_constraint.m_rel_pos1_cross_normal.dot(bodyA->get_angular_velocity() + external_torque_impulseA);
        real vel2_dotn = solver_constraint.m_contact_normal_2.dot(bodyB->get_linear_velocity() + external_force_impulseB) +
            solver_constraint.m_rel_pos2_cross_normal.dot(bodyB->get_angular_velocity() + external_torque_impulseB);
        real rel_vel = vel1_dotn + vel2_dotn;
        real restitution = 0.f;
        real positional_error = solver_constraint.m_rhs;
        // TODO:
        real velocity_error = restitution - rel_vel * info2.m_damping;
        real penetration_impulse = positional_error * solver_constraint.m_jac_diag_AB_inv;
        real velocity_impulse = velocity_error * solver_constraint.m_jac_diag_AB_inv;
        solver_constraint.m_rhs = penetration_impulse + velocity_impulse;
        solver_constraint.m_applied_impulse = 0.f;
    }
}

void b3SolverLxj::convert_contact(b3PersistentManifold* manifold, b3TimeStep* time_step) {
    b3Body* bodyA = manifold->get_bodyA();
    b3Body* bodyB = manifold->get_bodyB();

    int solver_body_idA = get_or_init_solver_body(*bodyA, time_step->m_dt);
    int solver_body_idB = get_or_init_solver_body(*bodyB, time_step->m_dt);

    b3SolverBody* solver_bodyA = &m_tmp_solver_bodies_pool[solver_body_idA];
    b3SolverBody* solver_bodyB = &m_tmp_solver_bodies_pool[solver_body_idB];

    // if bodyA and bodyB are static body
    if (solver_bodyA == nullptr || (solver_bodyA->m_inv_mass.is_zero() && (solver_bodyB == nullptr || solver_bodyB->m_inv_mass.is_zero()))) {
        return;
    }

    for (int j = 0; j < manifold->get_contact_point_count(); j++) {
        b3PersistentManifoldPoint& cp = manifold->get_cached_point(j);

        int friction_index = (int)m_tmp_solver_contact_constraint_pool.size();
        m_tmp_solver_contact_constraint_pool.emplace_back();
        b3SolverConstraint& solver_constraint = m_tmp_solver_contact_constraint_pool[friction_index];

        solver_constraint.m_solver_body_idA = solver_body_idA;
        solver_constraint.m_solver_body_idB = solver_body_idB;
        solver_constraint.m_original_contact_point = &cp;

        const b3Vec3r& posA = cp.m_position_world_on_A;
        const b3Vec3r& posB = cp.m_position_world_on_B;

        // 从物体质心到接触点的向量
        b3Vec3r rel_posA = posA - bodyA->get_position();
        b3Vec3r rel_posB = posB - bodyB->get_position();

        // 接触点处的速度
        b3Vec3r velA, velB;
        solver_bodyA->get_velocity_in_local_point_no_delta(rel_posA, velA);
        solver_bodyB->get_velocity_in_local_point_no_delta(rel_posB, velB);

        // 相对速度, 从物体B看物体A
        b3Vec3r vel = velA - velB;
        real rel_vel = cp.m_normal_world_on_B.dot(vel);

        real relaxation;
        setup_contact_constraint(solver_constraint, solver_body_idA, solver_body_idB, cp, time_step, relaxation, rel_posA, rel_posB);

        // setup for friction constraints, include tangent1 and tangent2, and rolling friction
        solver_constraint.m_friction_index = (int)m_tmp_solver_contact_friction_constraint_pool.size();

        if (cp.m_rolling_friction > 0.f) {
            add_torsional_friction_constraint(cp.m_normal_world_on_B, solver_body_idA, solver_body_idB, friction_index, cp,
                                              cp.m_spinning_friction);
            b3Vec3r axis0, axis1;
            b3_plane_space(cp.m_normal_world_on_B, axis0, axis1);
            axis0.normalized();
            axis1.normalized();

            if (axis0.length() > 0.001) {
                add_torsional_friction_constraint(axis0, solver_body_idA, solver_body_idB, friction_index, cp,
                                                  cp.m_rolling_friction);
            }
            if (axis1.length() > 0.001) {
                add_torsional_friction_constraint(axis1, solver_body_idA, solver_body_idB, friction_index, cp,
                                                  cp.m_rolling_friction);
            }
        }

        cp.m_lateral_friction_dir1 = vel - cp.m_normal_world_on_B * rel_vel;
        real lat_rel_vel = cp.m_lateral_friction_dir1.length2();
        if (lat_rel_vel > b3_real_epsilon) {
            cp.m_lateral_friction_dir1 /= b3_sqrt(lat_rel_vel);
            add_friction_constraint(cp.m_lateral_friction_dir1, solver_body_idA, solver_body_idB, friction_index, cp, rel_posA, rel_posB, relaxation);
            if (time_step->m_solve_use_2_friction_directions) {
                cp.m_lateral_friction_dir2 = cp.m_lateral_friction_dir1.cross(cp.m_normal_world_on_B);
                add_friction_constraint(cp.m_lateral_friction_dir2, solver_body_idA, solver_body_idB, friction_index, cp, rel_posA, rel_posB, relaxation);
            }
        } else {
            b3_plane_space(cp.m_normal_world_on_B, cp.m_lateral_friction_dir1, cp.m_lateral_friction_dir2);
            add_friction_constraint(cp.m_lateral_friction_dir1, solver_body_idA, solver_body_idB, friction_index, cp, rel_posA, rel_posB, relaxation);
            if (time_step->m_solve_use_2_friction_directions) {
                add_friction_constraint(cp.m_lateral_friction_dir2, solver_body_idA, solver_body_idB, friction_index, cp, rel_posA, rel_posB, relaxation);
            }
        }
    }
}


void b3SolverLxj::setup_friction_constraint(b3SolverConstraint& solver_constraint, const b3Vec3r& normal_axis, int solver_body_idA, int solver_body_idB,
    b3PersistentManifoldPoint& cp, const b3Vec3r& rel_posA, const b3Vec3r& rel_posB, real relaxation) {

    b3SolverBody& solver_bodyA = m_tmp_solver_bodies_pool[solver_body_idA];
    b3SolverBody& solver_bodyB = m_tmp_solver_bodies_pool[solver_body_idB];

    b3Body* bodyA = solver_bodyA.m_original_body;
    b3Body* bodyB = solver_bodyB.m_original_body;

    solver_constraint.m_solver_body_idA = solver_body_idA;
    solver_constraint.m_solver_body_idB = solver_body_idB;

    solver_constraint.m_friction = cp.m_friction;
    solver_constraint.m_original_contact_point = &cp;

    solver_constraint.m_applied_impulse = 0.f;
    solver_constraint.m_applied_push_impulse = 0.f;

    if (bodyA) {
        solver_constraint.m_contact_normal_1 = normal_axis;
        b3Vec3r torque_axis = rel_posA.cross(solver_constraint.m_contact_normal_1);
        solver_constraint.m_rel_pos1_cross_normal = torque_axis;
        solver_constraint.m_angular_componentA = bodyA->get_inv_inertia_tensor_world() * torque_axis;
    } else {
        solver_constraint.m_contact_normal_1.set_zero();
        solver_constraint.m_rel_pos1_cross_normal.set_zero();
        solver_constraint.m_angular_componentA.set_zero();
    }
    if (bodyB) {
        solver_constraint.m_contact_normal_2 = -normal_axis;
        b3Vec3r torque_axis = rel_posB.cross(solver_constraint.m_contact_normal_2);
        solver_constraint.m_rel_pos2_cross_normal = torque_axis;
        solver_constraint.m_angular_componentB = bodyB->get_inv_inertia_tensor_world() * torque_axis;
    } else {
        solver_constraint.m_contact_normal_2.set_zero();
        solver_constraint.m_rel_pos2_cross_normal.set_zero();
        solver_constraint.m_angular_componentB.set_zero();
    }

    b3Vec3r vec;
    real denom0 = 0.f, denom1 = 0.f;
    if (bodyA) {
        vec = (solver_constraint.m_angular_componentA).cross(rel_posA);
        denom0 = bodyA->get_inv_mass() + normal_axis.dot(vec);
    }
    if (bodyB) {
        vec = (-solver_constraint.m_angular_componentB).cross(rel_posB);
        denom1 = bodyB->get_inv_mass() + normal_axis.dot(vec);
    }
    solver_constraint.m_jac_diag_AB_inv = relaxation / (denom0 + denom1);

    real vel1_dotn = solver_constraint.m_contact_normal_1.dot(bodyA ? solver_bodyA.m_linear_velocity + solver_bodyA.m_external_force_impulse : b3Vec3r::zero())
        + solver_constraint.m_rel_pos1_cross_normal.dot(bodyA ? solver_bodyA.m_angular_velocity : b3Vec3r::zero());
    real vel2_dotn = solver_constraint.m_contact_normal_2.dot(bodyB ? solver_bodyB.m_linear_velocity + solver_bodyB.m_external_force_impulse : b3Vec3r::zero())
        + solver_constraint.m_rel_pos2_cross_normal.dot(bodyB ? solver_bodyB.m_angular_velocity : b3Vec3r::zero());
    real rel_vel = vel1_dotn + vel2_dotn;

    real velocity_error = -rel_vel;
    real velocity_impulse = velocity_error * solver_constraint.m_jac_diag_AB_inv;

    solver_constraint.m_rhs = velocity_impulse;
    solver_constraint.m_rhs_penetration = 0.f;
    solver_constraint.m_cfm = 0.f;
    solver_constraint.m_lower_limit = -solver_constraint.m_friction;
    solver_constraint.m_upper_limit = solver_constraint.m_friction;
}


b3SolverConstraint& b3SolverLxj::add_friction_constraint(const b3Vec3r& normal_axis, int solver_body_idA, int solver_body_idB, int friction_index,
    b3PersistentManifoldPoint& cp, const b3Vec3r& rel_posA, const b3Vec3r& rel_posB, real relaxation) {

    m_tmp_solver_contact_friction_constraint_pool.emplace_back();
    b3SolverConstraint& solver_constraint = m_tmp_solver_contact_friction_constraint_pool.back();
    solver_constraint.m_friction_index = friction_index;
    setup_friction_constraint(solver_constraint, normal_axis, solver_body_idA, solver_body_idB, cp, rel_posA, rel_posB, relaxation);
    return solver_constraint;
}

void b3SolverLxj::setup_torsional_friction_constraint(b3SolverConstraint& solver_constraint, const b3Vec3r& normal_axis1, int solver_body_idA,
    int solver_body_idB, b3PersistentManifoldPoint& cp, real torsional_friction) {

    b3Vec3r normal_axis = b3Vec3r::zero();

    solver_constraint.m_contact_normal_1 = normal_axis;
    solver_constraint.m_contact_normal_2 = -normal_axis;

    b3SolverBody& solver_bodyA = m_tmp_solver_bodies_pool[solver_body_idA];
    b3SolverBody& solver_bodyB = m_tmp_solver_bodies_pool[solver_body_idB];
    b3Body* bodyA = solver_bodyA.m_original_body;
    b3Body* bodyB = solver_bodyB.m_original_body;

    solver_constraint.m_solver_body_idA = solver_body_idA;
    solver_constraint.m_solver_body_idB = solver_body_idB;

    solver_constraint.m_friction = torsional_friction;
    solver_constraint.m_original_contact_point = &cp;
    solver_constraint.m_applied_impulse = 0.f;
    solver_constraint.m_applied_push_impulse = 0.f;

    solver_constraint.m_rel_pos1_cross_normal = -normal_axis1;
    solver_constraint.m_angular_componentA = bodyA ? bodyA->get_inv_inertia_tensor_world() * (-normal_axis1) : b3Vec3r::zero();

    solver_constraint.m_rel_pos2_cross_normal = normal_axis1;
    solver_constraint.m_angular_componentB = bodyB ? bodyB->get_inv_inertia_tensor_world() * normal_axis1 : b3Vec3r::zero();

    b3Vec3r iMJaA = bodyA ? bodyA->get_inv_inertia_tensor_world() * solver_constraint.m_rel_pos1_cross_normal : b3Vec3r::zero();
    b3Vec3r iMJaB = bodyB ? bodyB->get_inv_inertia_tensor_world() * solver_constraint.m_rel_pos2_cross_normal : b3Vec3r::zero();
    real sum = 0;
    sum += iMJaA.dot(solver_constraint.m_rel_pos1_cross_normal);
    sum += iMJaB.dot(solver_constraint.m_rel_pos2_cross_normal);
    solver_constraint.m_jac_diag_AB_inv = real(1.0) / sum;

    real velA_dotn = solver_constraint.m_contact_normal_1.dot(bodyA ? solver_bodyA.m_linear_velocity + solver_bodyA.m_external_force_impulse : b3Vec3r::zero())
        + solver_constraint.m_rel_pos1_cross_normal.dot(bodyA ? solver_bodyA.m_angular_velocity : b3Vec3r::zero());
    real velB_dotn = solver_constraint.m_contact_normal_2.dot(bodyB ? solver_bodyB.m_linear_velocity + solver_bodyB.m_external_force_impulse : b3Vec3r::zero())
        + solver_constraint.m_rel_pos2_cross_normal.dot(bodyB ? solver_bodyB.m_angular_velocity : b3Vec3r::zero());
    real rel_vel = velA_dotn + velB_dotn;
    real velocity_error = -rel_vel;
    real velocity_impulse = velocity_error * solver_constraint.m_jac_diag_AB_inv;
    solver_constraint.m_rhs = velocity_impulse;
    solver_constraint.m_cfm = 0;
    solver_constraint.m_lower_limit = -solver_constraint.m_friction;
    solver_constraint.m_upper_limit = solver_constraint.m_friction;
}

b3SolverConstraint& b3SolverLxj::add_torsional_friction_constraint(const b3Vec3r& normal_axis, int solver_body_idA,
    int solver_body_idB, int friction_index, b3PersistentManifoldPoint& cp, real torsional_friction) {

    m_tmp_solver_rolling_friction_constraint_pool.emplace_back();
    b3SolverConstraint& solver_constraint = m_tmp_solver_rolling_friction_constraint_pool.back();
    solver_constraint.m_friction_index = friction_index;
    setup_torsional_friction_constraint(solver_constraint, normal_axis, solver_body_idA, solver_body_idB, cp, torsional_friction);
    return solver_constraint;
}

void b3SolverLxj::setup_contact_constraint(b3SolverConstraint& solver_constraint, int solver_body_idA, int solver_body_idB,
    b3PersistentManifoldPoint& cp, const b3TimeStep* time_step, real& relaxation, const b3Vec3r& rel_posA, const b3Vec3r& rel_posB) {

    b3SolverBody* bodyA = &m_tmp_solver_bodies_pool[solver_body_idA];
    b3SolverBody* bodyB = &m_tmp_solver_bodies_pool[solver_body_idB];

    b3Body* rbA = bodyA->m_original_body;
    b3Body* rbB = bodyB->m_original_body;

    relaxation = time_step->m_sor;

    // cfm = 1/ (dt * kp + kd)
    // erp = dt * kp / (dt * kp + kd)
    real cfm = time_step->m_global_cfm * time_step->m_inv_dt;
    real erp = time_step->m_erp;

    b3Vec3r torque_axisA = rel_posA.cross(cp.m_normal_world_on_B);
    solver_constraint.m_angular_componentA = rbA ? rbA->get_inv_inertia_tensor_world() * torque_axisA : b3Vec3r::zero();
    b3Vec3r torque_axisB = rel_posB.cross(cp.m_normal_world_on_B);
    solver_constraint.m_angular_componentB = rbB ? rbB->get_inv_inertia_tensor_world() * (-torque_axisB) : b3Vec3r::zero();

    b3Vec3r vec;
    real denom0 = 0.f, denom1 = 0.f;
    if (rbA) {
        vec = (solver_constraint.m_angular_componentA).cross(rel_posA);
        denom0 = rbA->get_inv_mass() + cp.m_normal_world_on_B.dot(vec);
    }
    if (rbB) {
        vec = (-solver_constraint.m_angular_componentB).cross(rel_posB);
        denom1 = rbB->get_inv_mass() + cp.m_normal_world_on_B.dot(vec);
    }

    solver_constraint.m_jac_diag_AB_inv = relaxation / (denom0 + denom1 + cfm);

    if (rbA) {
        solver_constraint.m_contact_normal_1 = cp.m_normal_world_on_B;
        solver_constraint.m_rel_pos1_cross_normal = torque_axisA;
    } else {
        solver_constraint.m_contact_normal_1.set_zero();
        solver_constraint.m_rel_pos1_cross_normal.set_zero();
    }
    if (rbB) {
        solver_constraint.m_contact_normal_2 = -cp.m_normal_world_on_B;
        solver_constraint.m_rel_pos2_cross_normal = -torque_axisB;
    } else {
        solver_constraint.m_contact_normal_2.set_zero();
        solver_constraint.m_rel_pos2_cross_normal.set_zero();
    }

    // ware start
    if (time_step->m_warm_starting) {
        solver_constraint.m_applied_impulse = cp.m_applied_impulse * time_step->m_warm_starting_factor;
        if (rbA) {
            bodyA->internal_apply_impulse(solver_constraint.m_contact_normal_1 * bodyA->internal_get_inv_mass(), solver_constraint.m_angular_componentA, solver_constraint.m_applied_impulse);
        }
        if (rbB) {
            bodyB->internal_apply_impulse(-solver_constraint.m_contact_normal_2 * bodyB->internal_get_inv_mass(), -solver_constraint.m_angular_componentB, -solver_constraint.m_applied_impulse);
        }
    } else {
        solver_constraint.m_applied_impulse = 0.f;
    }
    solver_constraint.m_applied_push_impulse = 0.f;
    solver_constraint.m_friction = cp.m_friction;

    b3Vec3r velA = rbA ? rbA->get_velocity_in_local_point(rel_posA) : b3Vec3r::zero();
    b3Vec3r velB = rbB ? rbB->get_velocity_in_local_point(rel_posB) : b3Vec3r::zero();
    b3Vec3r vel = velA - velB;
    real rel_vel = cp.m_normal_world_on_B.dot(vel);

    real restitution = restitution_curve(rel_vel, cp.m_restitution, time_step->m_restitution_velocity_threshold);

    b3Vec3r external_force_impulseA = rbA ? bodyA->m_external_force_impulse : b3Vec3r::zero();
    b3Vec3r external_torque_impulseA = rbA ? bodyA->m_external_torque_impulse : b3Vec3r::zero();
    b3Vec3r external_force_impulseB = rbB ? bodyB->m_external_force_impulse : b3Vec3r::zero();
    b3Vec3r external_torque_impulseB = rbB ? bodyB->m_external_torque_impulse : b3Vec3r::zero();

    real velA_dot_n = solver_constraint.m_contact_normal_1.dot(bodyA->m_linear_velocity + external_force_impulseA) +
        solver_constraint.m_rel_pos1_cross_normal.dot(bodyA->m_angular_velocity + external_torque_impulseA);
    real velB_dot_n = solver_constraint.m_contact_normal_2.dot(bodyB->m_linear_velocity + external_force_impulseB) +
        solver_constraint.m_rel_pos2_cross_normal.dot(bodyB->m_angular_velocity + external_torque_impulseB);

    rel_vel = velA_dot_n + velB_dot_n;

    real position_error = 0.f;
    real velocity_error = restitution - rel_vel;

    real penetration = cp.get_distance();
    if (penetration > 0) {
        position_error = 0;
        velocity_error -= penetration * time_step->m_inv_dt;
    } else {
        position_error = -penetration * erp * time_step->m_inv_dt;
    }

    real penetration_impulse = position_error * solver_constraint.m_jac_diag_AB_inv;
    real velocity_impulse = velocity_error * solver_constraint.m_jac_diag_AB_inv;

    if (penetration > time_step->m_split_impulse_penetration_threshold) {
        solver_constraint.m_rhs = velocity_impulse + penetration_impulse;
        solver_constraint.m_rhs_penetration = 0;
    } else {
        solver_constraint.m_rhs = velocity_impulse;
        solver_constraint.m_rhs_penetration = penetration_impulse;
    }
    solver_constraint.m_cfm = cfm * solver_constraint.m_jac_diag_AB_inv;
    solver_constraint.m_lower_limit = 0;
    solver_constraint.m_upper_limit = 1e10f;
}


int b3SolverLxj::get_or_init_solver_body(b3Body &body, real dt) {
    int solver_body_idA = -1;

    if (body.get_companion_id() >= 0) {
        solver_body_idA = body.get_companion_id();
    } else {
        if (body.get_inv_mass() > real(0.0)) {
            solver_body_idA = (int)m_tmp_solver_bodies_pool.size();
            b3SolverBody solver_body;
            init_solver_body(solver_body, &body, dt);
            m_tmp_solver_bodies_pool.emplace_back(solver_body);
            body.set_companion_id(solver_body_idA);
        } else {
            if (m_fixed_body_id < 0) {
                m_fixed_body_id = (int)m_tmp_solver_bodies_pool.size();
                b3SolverBody fixed_body;
                init_solver_body(fixed_body, nullptr, dt);
                m_tmp_solver_bodies_pool.emplace_back(fixed_body);
            }
            return m_fixed_body_id;
        }
    }

    return solver_body_idA;
}

void b3SolverLxj::init_solver_body(b3SolverBody& solver_body, b3Body* body, real dt) {

    solver_body.internal_get_delta_linear_velocity().set_zero();
    solver_body.internal_get_delta_angular_velocity().set_zero();
    solver_body.internal_get_push_velocity().set_zero();
    solver_body.internal_get_turn_velocity().set_zero();

    if (body == nullptr) {
        solver_body.m_world_transform.set_identity();
        solver_body.internal_set_inv_mass(b3Vec3r::zero());
        solver_body.m_original_body = nullptr;
        solver_body.m_linear_velocity.set_zero();
        solver_body.m_angular_velocity.set_zero();
        solver_body.m_external_force_impulse.set_zero();
        solver_body.m_external_torque_impulse.set_zero();
    } else {
        solver_body.m_world_transform = body->get_world_transform();
        solver_body.internal_set_inv_mass(b3Vec3r(body->get_inv_mass(), body->get_inv_mass(), body->get_inv_mass()));
        solver_body.m_original_body = body;
        solver_body.m_linear_velocity = body->get_linear_velocity();
        solver_body.m_angular_velocity = body->get_angular_velocity();

        solver_body.m_linear_velocity *= 1.0f / (1.0f + dt * body->get_linear_damping());
        solver_body.m_angular_velocity *= 1.0f / (1.0f + dt * body->get_angular_damping());

        // 外力和外力矩
        solver_body.m_external_force_impulse = body->get_force() * body->get_inv_mass() * dt;

        const b3Mat33r& R = solver_body.m_world_transform.rotation_matrix();
        b3Mat33r world_inertia = R * body->get_inv_inertia() * R.transpose();
        solver_body.m_external_torque_impulse = body->get_torque() * body->get_inv_inertia_tensor_world() * dt;
        // solver_body.m_external_torque_impulse = body->get_torque() * body->get_inv_inertia() * dt;
    }
}