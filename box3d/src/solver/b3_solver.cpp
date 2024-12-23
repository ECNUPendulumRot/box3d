
#include "solver/b3_solver.hpp"

#include "dynamics/b3_island.hpp"
#include "dynamics/b3_body.hpp"
#include "collision/b3_collision.hpp"
#include "collision/b3_contact.hpp"
#include "common/b3_time_step.hpp"
#include "common/b3_block_allocator.hpp"
#include "collision/b3_fixture.hpp"

#include "common/b3_draw.hpp"

#include "spdlog/spdlog.h"

void b3Solver::init(b3BlockAllocator *block_allocator, b3Island *island, b3TimeStep *step)
{
    m_timestep = step;
    m_block_allocator = block_allocator;

    m_contact_count = island->get_contacts_count();
    m_contacts = island->get_contacts();

    b3_assert(m_contact_count >= 0);

    // allocate memory for all bodies and contacts of every island.
    // and we need extra arrays to store the velocity and position of every body.
    // after solver all contacts and friction constraints, we copy the results back to bodies.

    // The number of velocity constraints is same to the number of contacts.
    void* memory;
    if(m_contact_count > 0) {
        memory = m_block_allocator->allocate(m_contact_count * sizeof(b3ContactVelocityConstraint));
        m_velocity_constraints = new (memory) b3ContactVelocityConstraint;
    } else {
        m_velocity_constraints = nullptr;
    }

    m_body_count = island->get_body_count();
    b3_assert(m_body_count > 0);
    m_bodies = island->get_bodies();

    memory = m_block_allocator->allocate(m_body_count * sizeof(b3Vec3r));
    m_ps = new (memory) b3Vec3r;

    memory = m_block_allocator->allocate(m_body_count * sizeof(b3Quaternionr));
    m_qs = new (memory) b3Quaternionr;

    memory = m_block_allocator->allocate(m_body_count * sizeof(b3Vec3r));
    m_vs = new (memory) b3Vec3r;

    memory = m_block_allocator->allocate(m_body_count * sizeof(b3Vec3r));
    m_ws = new (memory) b3Vec3r;

    for(int32 i = 0; i < m_body_count; ++i) {
        b3Body* b = m_bodies[i];
        m_ps[i] = b->get_position();
        m_qs[i] = b->get_quaternion();
        m_vs[i] = b->get_linear_velocity();
        m_ws[i] = b->get_angular_velocity();
    }

    ////////////////////////////////////////////////////////////////////////////////

    for(int32 i = 0; i < m_contact_count; ++i) {
        b3Contact* contact = m_contacts[i];

        b3Fixture *fixture_a = contact->get_fixture_a();
        b3Fixture *fixture_b = contact->get_fixture_b();

        b3Body *body_a = fixture_a->get_body();
        b3Body *body_b = fixture_b->get_body();

        b3Manifold *manifold = contact->get_manifold();

        int32 point_count = manifold->m_point_count;

        b3_assert(point_count > 0);

        b3ContactVelocityConstraint* vc = m_velocity_constraints + i;
        vc->m_restitution = contact->get_restitution();
        vc->m_friction = contact->get_friction();
        vc->m_rolling_friction = contact->get_rolling_friction();
        vc->m_spinning_friction = contact->get_spinning_friction();

        vc->m_contact_index = i;
        vc->m_point_count = point_count;

        vc->m_normal = manifold->m_local_normal;
        b3_plane_space(vc->m_normal, vc->m_tangent1, vc->m_tangent2);

        vc->m_index_a = body_a->get_island_index();
        vc->m_mass_a = body_a->get_mass();
        vc->m_inv_mass_a = body_a->get_inv_mass();

        b3Mat33r R_a;
        R_a.set_rotation(body_a->get_quaternion());

        vc->m_I_a = R_a * body_a->get_inertia() * R_a.transpose();
        vc->m_inv_I_a = R_a.transpose() * body_a->get_inv_inertia() * R_a;

        vc->m_index_b = body_b->get_island_index();
        vc->m_mass_b = body_b->get_mass();
        vc->m_inv_mass_b = body_b->get_inv_mass();

        b3Mat33r R_b;
        R_b.set_rotation(body_b->get_quaternion());

        vc->m_I_b = R_b * body_b->get_inertia() * R_b.transpose();
        vc->m_inv_I_b = R_b.transpose() * body_b->get_inv_inertia() * R_b;

        vc->m_penetration = manifold->m_penetration;

        b3Transformr xf_a(body_a->get_position(), body_a->get_quaternion());
        b3Transformr xf_b(body_b->get_position(), body_b->get_quaternion());

        b3Vec3r center_a = xf_a.transform(body_a->get_local_center());
        b3Vec3r center_b = xf_b.transform(body_b->get_local_center());

        for (int32 j = 0; j < point_count; j++) {
            b3VelocityConstraintPoint *vcp = vc->m_points + j;
            b3ManifoldPoint *manifold_point = manifold->m_points + j;

            vcp->m_ra = manifold_point->m_local_point - center_a;
            vcp->m_rb = manifold_point->m_local_point - center_b;
            // vcp->m_rhs_penetration = manifold->m_penetration;
            // TODO: warm start
        }

        vc->init();
    }

}


void b3Solver::write_states_back()
{
    for(int32 i = 0; i < m_body_count; ++i) {
        b3Body* b = m_bodies[i];

        b->set_position(m_ps[i]);
        b->set_quaternion(m_qs[i]);
        b->set_linear_velocity(m_vs[i]);
        b->set_angular_velocity(m_ws[i]);
        // TODO: if we need SynchronizeTransform() ?
    }

//    for(int32 i = 0; i < m_body_count; i++) {
//        if(m_bodies[i]->get_type() == b3BodyType::b3_dynamic_body) {
//            spdlog::log(spdlog::level::info, "count: {}, velocity: {}, {}, {}, {}, {}, {}",
//                        m_contact_count,
//                        m_bodies[i]->get_linear_velocity().x,
//                        m_bodies[i]->get_linear_velocity().y,
//                        m_bodies[i]->get_linear_velocity().z,
//                        m_bodies[i]->get_angular_velocity().x,
//                        m_bodies[i]->get_angular_velocity().y,
//                        m_bodies[i]->get_angular_velocity().z
//            );
//        }
//    }
}


int b3Solver::solve() {

    init_velocity_constraints();

    // collision
    for(int32 i = 0; i < m_timestep->m_velocity_iterations; ++i) {
        solve_velocity_constraints(true);
    }

    // velocity update
    for(int32 i = 0; i < m_body_count; ++i) {
        b3Body *b = m_bodies[i];
        b3Vec3r v = m_vs[i];
        b3Vec3r w = m_ws[i];

        v += m_timestep->m_dt * b->get_inv_mass() * (b->get_force() + b->get_gravity());
        w += m_timestep->m_dt * b->get_inv_inertia() * b->get_torque();

        // Apply damping.
        // ODE: dv/dt + c * v = 0
        // Solution: v(t) = v0 * exp(-c * t)
        // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
        // v2 = exp(-c * dt) * v1
        // Pade approximation:
        // v2 = v1 * 1 / (1 + c * dt)
        // TODO: make this params become class members.
//        v *= 1.0f / (1.0f + m_timestep->m_dt * 0.1);
//        w *= 1.0f / (1.0f + m_timestep->m_dt * 0.1);

        m_vs[i] = v;
        m_ws[i] = w;
    }

    // contact
    for (int32 i = 0; i < m_timestep->m_velocity_iterations; ++i) {
        solve_velocity_constraints(false);
    }

    for (int32 i = 0; i < m_body_count; ++i) {
        m_ps[i] = m_ps[i] + m_vs[i] * m_timestep->m_dt;
        if (m_bodies[i]->get_type() == b3BodyType::b3_dynamic_body) {
            m_ws[i][0] = 0.01f;
        }
        m_qs[i] = m_qs[i] + real(0.5) * m_timestep->m_dt * b3Quaternionr(0, m_ws[i]) * m_qs[i];
        m_qs[i].normalize();
        m_ws[i][0] = 0;
    }

    for (int32 i = 0; i < m_body_count; i++) {
        if (m_bodies[i]->get_type() == b3BodyType::b3_dynamic_body) {
            spdlog::log(spdlog::level::info, "position: {}, {}, {}, rotation: {}, {}, {}, {}", m_ps[i].x, m_ps[i].y, m_ps[i].z, m_qs[i].m_x, m_qs[i].m_y, m_qs[i].m_z, m_qs[i].m_w);
            spdlog::log(spdlog::level::info, "velocity: {}, {}, {}, w: {}, {}, {}", m_vs[i].x, m_vs[i].y, m_vs[i].z, m_ws[i].x, m_ws[i].y, m_ws[i].z);
        }
    }

    // correct_penetration();

    // copy state buffers back to the bodies.
    write_states_back();

    return 0;
}


void b3Solver::solve_velocity_constraints(bool is_collision)
{
    for (int32 i = 0; i < m_contact_count; ++i) {
        b3ContactVelocityConstraint *vc = m_velocity_constraints + i;

        b3Vec3r v_a = m_vs[vc->m_index_a];
        b3Vec3r w_a = m_ws[vc->m_index_a];

        b3Vec3r v_b = m_vs[vc->m_index_b];
        b3Vec3r w_b = m_ws[vc->m_index_b];

        // Follow the example of Box2d and prioritize friction because penetration is more important.
        for (int32 j = 0; j < vc->m_point_count; j++) {
            b3VelocityConstraintPoint* vcp = vc->m_points + j;

            b3Vec3r v_rel = v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra);

            real vt1 = v_rel.dot(vc->m_tangent1);
            real vt2 = v_rel.dot(vc->m_tangent2);

            float tangent1_lambda = vcp->m_tangent1_mass * (-vt1);
            float tangent2_lambda = vcp->m_tangent2_mass * (-vt2);

            real max_friction;
            if(is_collision) {
                max_friction = vc->m_friction * vcp->m_normal_collision_impulse;
                float new_tangent1_lambda = b3_clamp(vcp->m_tangent1_collision_impulse + tangent1_lambda, -max_friction, max_friction);
                tangent1_lambda = new_tangent1_lambda - vcp->m_tangent1_collision_impulse;
                vcp->m_tangent1_collision_impulse = new_tangent1_lambda;

                float new_tangent2_lambda = b3_clamp(vcp->m_tangent2_collision_impulse + tangent2_lambda, -max_friction, max_friction);
                tangent2_lambda = new_tangent2_lambda - vcp->m_tangent2_collision_impulse;
                vcp->m_tangent2_collision_impulse = new_tangent2_lambda;
            } else {
                max_friction = vc->m_friction * vcp->m_normal_contact_impulse;

                float new_tangent1_lambda = b3_clamp(vcp->m_tangent1_contact_impulse + tangent1_lambda, -max_friction, max_friction);
                tangent1_lambda = new_tangent1_lambda - vcp->m_tangent1_contact_impulse;
                vcp->m_tangent1_contact_impulse = new_tangent1_lambda;

                float new_tangent2_lambda = b3_clamp(vcp->m_tangent2_contact_impulse + tangent2_lambda, -max_friction, max_friction);
                tangent2_lambda = new_tangent2_lambda - vcp->m_tangent2_contact_impulse;
                vcp->m_tangent2_contact_impulse = new_tangent2_lambda;
            }

            b3Vec3r impulse = tangent1_lambda * vc->m_tangent1 + tangent2_lambda * vc->m_tangent2;

            v_a = v_a - vc->m_inv_mass_a * impulse;
            w_a = w_a - vc->m_inv_I_a * vcp->m_ra.cross(impulse);
            v_b = v_b + vc->m_inv_mass_b * impulse;
            w_b = w_b + vc->m_inv_I_b * vcp->m_rb.cross(impulse);
        }

        real total_impulse = 0;

        for (int32 j = 0; j < vc->m_point_count; ++j) {
            b3VelocityConstraintPoint *vcp = vc->m_points + j;

            b3Vec3r v_rel = v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra);
            real rhs = -v_rel.dot(vc->m_normal);

            // TODO:
            real lambda = 0;
            if (is_collision) {
                lambda = vcp->m_normal_mass * (rhs + vcp->m_rhs_restitution_velocity);
                real new_impulse = b3_max(vcp->m_normal_collision_impulse + lambda, (real) 0.0);
                lambda = new_impulse - vcp->m_normal_collision_impulse;
                vcp->m_normal_collision_impulse = new_impulse;
            } else {
                lambda = vcp->m_normal_mass * rhs;
                real new_impulse = b3_max(vcp->m_normal_contact_impulse + lambda, (real) 0.0);
                lambda = new_impulse - vcp->m_normal_contact_impulse;
                vcp->m_normal_contact_impulse = new_impulse;
            }

            // apply normal Impulse
            b3Vec3r impulse = lambda * vc->m_normal;

            total_impulse += lambda;

            v_a = v_a - vc->m_inv_mass_a * impulse;
            v_b = v_b + vc->m_inv_mass_b * impulse;
            w_a = w_a - vc->m_inv_I_a * vcp->m_ra.cross(impulse);
            w_b = w_b + vc->m_inv_I_b * vcp->m_rb.cross(impulse);
        }

        apply_spinning_and_rolling_friction1(vc, w_a, w_b, total_impulse);

        m_vs[vc->m_index_a] = v_a;
        m_vs[vc->m_index_b] = v_b;
        m_ws[vc->m_index_a] = w_a;
        m_ws[vc->m_index_b] = w_b;
    }
}


void b3Solver::apply_spinning_and_rolling_friction1(b3ContactVelocityConstraint* vc, b3Vec3r& w_a, b3Vec3r& w_b, real total_impulse)
{
    vc->m_total_normal_impulse += total_impulse;

    if (vc->m_spinning_friction < b3_real_epsilon && vc->m_rolling_friction < b3_real_epsilon) {
        return;
    }

    static auto helper_func = [](real& Iw_x, real& used, real max) {
        if (b3_abs(Iw_x) < b3_real_epsilon) {
            Iw_x = 0;
            return;
        }
        real now_using = b3_clamp(-Iw_x, -max - used, max - used);
        used += now_using;
        Iw_x += now_using;
    };

    b3Vec3r Iw_a = vc->m_I_a * w_a;
    b3Vec3r Iw_b = vc->m_I_b * w_b;

    real Iw_a_n = Iw_a.dot(vc->m_normal);
    real Iw_b_n = Iw_b.dot(vc->m_normal);
    real Iw_a_t1 = Iw_a.dot(vc->m_tangent1);
    real Iw_a_t2 = Iw_a.dot(vc->m_tangent2);
    real Iw_b_t1 = Iw_b.dot(vc->m_tangent1);
    real Iw_b_t2 = Iw_b.dot(vc->m_tangent2);

    if (vc->m_spinning_friction > 0) {
        real spinning_friction = vc->m_spinning_friction * vc->m_total_normal_impulse;

        // spinning
        helper_func(Iw_a_n, vc->m_used_spinning_friction_impulse_torqueA, spinning_friction);
        // spinning
        helper_func(Iw_b_n, vc->m_used_spinning_friction_impulse_torqueB, spinning_friction);
    }

    if (vc->m_rolling_friction > 0) {
        real rolling_friction = vc->m_rolling_friction * vc->m_total_normal_impulse;

        // rolling
        helper_func(Iw_a_t1, vc->m_used_rolling_friction_impulse_torqueA1, rolling_friction);
        helper_func(Iw_a_t2, vc->m_used_rolling_friction_impulse_torqueA2, rolling_friction);
        // rolling
        helper_func(Iw_b_t1, vc->m_used_rolling_friction_impulse_torqueB1, rolling_friction);
        helper_func(Iw_b_t2, vc->m_used_rolling_friction_impulse_torqueB2, rolling_friction);
    }

    Iw_a = Iw_a_n * vc->m_normal + Iw_a_t1 * vc->m_tangent1 + Iw_a_t2 * vc->m_tangent2;
    w_a = vc->m_inv_I_a * Iw_a;
    Iw_b = Iw_b_n * vc->m_normal + Iw_b_t1 * vc->m_tangent1 + Iw_b_t2 * vc->m_tangent2;
    w_b = vc->m_inv_I_b * Iw_b;
}


void b3Solver::apply_spinning_and_rolling_friction2(
    b3ContactVelocityConstraint *vc, b3Vec3r &w_a, b3Vec3r &w_b, real total_impulse)
{
    vc->m_total_normal_impulse += total_impulse;
    if (vc->m_spinning_friction < b3_real_epsilon && vc->m_rolling_friction < b3_real_epsilon) {
        return;
    }

    if (vc->m_total_normal_impulse > 0.f) {
        real upper_limit = vc->m_rolling_friction * vc->m_total_normal_impulse;
        real lower_limit = -upper_limit;
    }

    b3Vec3r relative_angular = w_b - w_a;
}


void b3Solver::correct_penetration()
{
    for (int32 i = 0; i < m_contact_count; ++i) {
        b3ContactVelocityConstraint *vc = m_velocity_constraints + i;

        // TODO: Check this way is useful. And angle?
        if (vc->m_penetration < 0) {
            b3Vec3r p_a = m_ps[vc->m_index_a];
            b3Vec3r p_b = m_ps[vc->m_index_b];

            b3Vec3r position_correction = vc->m_normal * vc->m_penetration;

            // at first, two bodies are not static.
            // if two bodies are dynamic, all need to fix penetration.
            // if one body is static, the dynamic body need to fix penetration * 2.
            // because when generate manifold, the penetration is equally distributed to two bodies.
            if (m_bodies[vc->m_index_a]->get_type() == b3BodyType::b3_dynamic_body &&
                m_bodies[vc->m_index_b]->get_type() == b3BodyType::b3_dynamic_body) {
                p_a += vc->m_normal * vc->m_penetration;
                p_b -= vc->m_normal * vc->m_penetration;
            } else if (m_bodies[vc->m_index_a]->get_type() == b3BodyType::b3_dynamic_body) {
                p_a += position_correction * real(2.0);
            } else if (m_bodies[vc->m_index_b]->get_type() == b3BodyType::b3_dynamic_body) {
                p_b -= position_correction * real(2.0);
            }

            m_ps[vc->m_index_a] = p_a;
            m_ps[vc->m_index_b] = p_b;

            // after fix penetration, the penetration should be zero.
            // TODO: we could fix part of penetration, but not all.
            vc->m_penetration = 0;
        }
    }
}


void b3Solver::init_velocity_constraints()
{
    for(int32 i = 0; i < m_contact_count; ++i) {
        b3ContactVelocityConstraint* vc = m_velocity_constraints + i;
        int32 index_a = vc->m_index_a;
        int32 index_b = vc->m_index_b;

        b3Vec3r v_a = m_vs[index_a];
        b3Vec3r w_a = m_ws[index_a];

        b3Vec3r v_b = m_vs[index_b];
        b3Vec3r w_b = m_ws[index_b];

        for (int j = 0; j < vc->m_point_count; ++j) {
            b3VelocityConstraintPoint *vcp = vc->m_points + j;

            // vcp->m_ra(b) is a contact point(from the center of body)
            b3Vec3r ra_n = vcp->m_ra.cross(vc->m_normal);
            b3Vec3r rb_n = vcp->m_rb.cross(vc->m_normal);

            // JM_INV_J
            // In box3d, a col vector multiply a matrix is the
            // transpose of the vector multiply the matrix.
            real effective_mass = vc->m_inv_mass_a + vc->m_inv_mass_b +
                                  (ra_n * vc->m_inv_I_a).dot(ra_n) +
                                  (rb_n * vc->m_inv_I_b).dot(rb_n);

            vcp->m_normal_mass = effective_mass > 0 ? (real)1.0 / effective_mass : 0;

            b3Vec3r ra_t1 = vcp->m_ra.cross(vc->m_tangent1);
            b3Vec3r rb_t1 = vcp->m_rb.cross(vc->m_tangent1);
            effective_mass = vc->m_inv_mass_a + vc->m_inv_mass_b + (ra_t1 * vc->m_inv_I_a).dot(ra_t1)
                             + (rb_t1 * vc->m_inv_I_b).dot(rb_t1);

            vcp->m_tangent1_mass = effective_mass > 0 ? real(1.0) / effective_mass : 0;

            b3Vec3r ra_t2 = vcp->m_ra.cross(vc->m_tangent2);
            b3Vec3r rb_t2 = vcp->m_rb.cross(vc->m_tangent2);
            effective_mass = vc->m_inv_mass_a + vc->m_inv_mass_b + (ra_t2 * vc->m_inv_I_a).dot(ra_t2)
                             + (rb_t2 * vc->m_inv_I_b).dot(rb_t2);
            vcp->m_tangent2_mass = effective_mass > 0 ? real(1.0) / effective_mass : 0;

            // 1. Mv+ = Mv + J^T * lambda ==> Jv+ = Jv + JM_invJ^T * lambda
            // 2. Jv+ = J(-ev)
            // ===> JM_invJ^T * lambda = -eJv - Jv
            real v_rel = vc->m_normal.dot(v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra));
            // m_rhs_restitution_velocity is eJv
            vcp->m_rhs_restitution_velocity = -vc->m_restitution * v_rel;

            vcp->m_normal_collision_impulse = 0.0;
            vcp->m_normal_contact_impulse = 0.0;
            vcp->m_tangent1_collision_impulse = 0.0;
            vcp->m_tangent1_contact_impulse = 0.0;
            vcp->m_tangent2_collision_impulse = 0.0;
            vcp->m_tangent2_contact_impulse = 0.0;
        }

        // TODO: if we have more than one contact point, then prepare the block solver.
    }
}


b3Solver::~b3Solver()
{
    clear();
}


void b3Solver::clear() {
    m_timestep = nullptr;
    m_contacts = nullptr;

    m_block_allocator->free(m_velocity_constraints, m_contact_count * sizeof(b3ContactVelocityConstraint));
    m_block_allocator->free(m_ps, m_body_count * sizeof(b3Vec3r));
    m_block_allocator->free(m_qs, m_body_count * sizeof(b3Quaternionr));
    m_block_allocator->free(m_vs, m_body_count * sizeof(b3Vec3r));
    m_block_allocator->free(m_ws, m_body_count * sizeof(b3Vec3r));

    m_block_allocator = nullptr;
}