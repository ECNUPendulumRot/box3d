
#include "solver/b3_solver_gr.hpp"

#include "dynamics/b3_island.hpp"
#include "dynamics/b3_body.hpp"


#include "collision/b3_fixture.hpp"
#include "collision/b3_contact.hpp"

#include "common/b3_time_step.hpp"
#include "common/b3_block_allocator.hpp"

#include "solver/b3_contact_constraint.hpp"
#include "solver/b3_lemke.hpp"

#include "spdlog/spdlog.h"


bool g_block_solve = false;

b3SolverGR::b3SolverGR(b3BlockAllocator *block_allocator, b3Island *island, b3TimeStep *step)
{
    init(block_allocator, island, step);
}


void b3SolverGR::init(b3BlockAllocator *block_allocator, b3Island *island, b3TimeStep *step)
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

        memory = m_block_allocator->allocate(m_contact_count * sizeof(b3ContactVelocityConstraint*));
        m_violated_constraints = new (memory) b3ContactVelocityConstraint*;
        m_violated_count = 0;
    } else {
        m_velocity_constraints = nullptr;
        m_violated_constraints = nullptr;
        m_violated_count = 0;
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

    ////////////////////////// Initialize Contact Constraints //////////////////////////

    for(int32 i = 0; i < m_contact_count; ++i) {

        b3Contact* contact = m_contacts[i];

        b3Fixture* fixture_a = contact->get_fixture_a();
        b3Fixture* fixture_b = contact->get_fixture_b();

        b3Body* body_a = fixture_a->get_body();
        b3Body* body_b = fixture_b->get_body();

        b3Manifold* manifold = contact->get_manifold();

        int32 point_count = manifold->m_point_count;

        b3_assert(point_count > 0);

        b3ContactVelocityConstraint* vc = m_velocity_constraints + i;
        vc->m_restitution = contact->get_restitution();
        vc->m_friction = contact->get_friction();
        vc->m_contact_index = i;
        vc->m_point_count = point_count;
        vc->m_restitution_threshold = contact->get_restitution_threshold();
        vc->m_normal = manifold->m_local_normal;

        vc->m_index_a = body_a->get_island_index();
        vc->m_mass_a = body_a->get_mass();
        vc->m_inv_mass_a = body_a->get_inv_mass();

        const b3Mat33r& R_a = body_a->get_quaternion().rotation_matrix();

        vc->m_I_a = R_a * body_a->get_inertia() * R_a.transpose();
        vc->m_inv_I_a = R_a.transpose() * body_a->get_inv_inertia() * R_a;

        vc->m_index_b = body_b->get_island_index();

        vc->m_mass_b = body_b->get_mass();
        vc->m_inv_mass_b = body_b->get_inv_mass();

        const b3Mat33r& R_b = body_b->get_quaternion().rotation_matrix();

        vc->m_I_b = R_b * body_b->get_inertia() * R_b.transpose();
        vc->m_inv_I_b = R_b.transpose() * body_b->get_inv_inertia() * R_b;

        vc->m_penetration = manifold->m_penetration;

        // the center of body in the world frame

        b3Transformr xf_a(body_a->get_position(), body_a->get_quaternion());
        b3Transformr xf_b(body_b->get_position(), body_b->get_quaternion());

        b3Vec3r center_a = xf_a.transform(body_a->get_local_center());
        b3Vec3r center_b = xf_b.transform(body_b->get_local_center());

        for (int32 j = 0; j < point_count; j++) {
            b3VelocityConstraintPoint* vcp = vc->m_points + j;
            b3ManifoldPoint* manifold_point = manifold->m_points + j;

            vcp->m_ra = manifold_point->m_local_point - center_a;
            vcp->m_rb = manifold_point->m_local_point - center_b;
        }
    }
}


void b3SolverGR::write_states_back()
{
    for(int32 i = 0; i < m_body_count; ++i) {
        m_bodies[i]->set_position(m_ps[i]);
        m_bodies[i]->set_quaternion(m_qs[i]);
        m_bodies[i]->set_linear_velocity(m_vs[i]);
        m_bodies[i]->set_angular_velocity(m_ws[i]);
    }
}


int b3SolverGR::solve(bool allow_sleep)
{
    // spdlog::info("|||||||||||||||||| Solve ||||||||||||||||||");

    for(int32 i = 0; i < m_body_count; ++i) {

        b3Body* b = m_bodies[i];

        b3Vec3r v = m_vs[i];
        b3Vec3r w = m_ws[i];

        v += m_timestep->m_dt * b->get_inv_mass() * (b->get_force() + b->get_gravity());
        w += m_timestep->m_dt * b->get_inv_inertia() * b->get_torque();

        m_vs[i] = v;
        m_ws[i] = w;
    }

    init_velocity_constraints();

    solve_velocity_constraints(m_timestep->m_velocity_iterations);

    for (int32 i = 0; i < m_body_count; ++i) {
        m_ps[i] = m_ps[i] + m_vs[i] * m_timestep->m_dt;
        m_qs[i] = m_qs[i] + real(0.5) * m_timestep->m_dt * b3Quaternionr(0, m_ws[i]) * m_qs[i];
        m_qs[i].normalize();
    }

    if (allow_sleep) {
        const float lin_tor_sqr = b3_linear_sleep_tolerance * b3_linear_sleep_tolerance;
        const float ang_tor_sqr = b3_angular_sleep_tolerance * b3_angular_sleep_tolerance;

        for (int32 i = 0; i < m_body_count; ++i) {
            b3Body* b = m_bodies[i];
            if (b->get_type() == b3BodyType::b3_static_body) {
                continue;
            }

            b3Vec3r lin_vel = m_vs[i];
            b3Vec3r ang_vel = m_ws[i];

            if (lin_vel.dot(lin_vel) < lin_tor_sqr && ang_vel.dot(ang_vel) < ang_tor_sqr) {
                b->set_awake(false);
            } else {
                b->set_awake(true);
            }
        }
    }

    // copy state buffers back to the bodies.
    write_states_back();

    return 0;
}


void b3SolverGR::init_velocity_constraints()
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

            b3Vec3r ra_n = vcp->m_ra.cross(vc->m_normal);
            b3Vec3r rb_n = vcp->m_rb.cross(vc->m_normal);

            // JM_INV_J
            // In box3d, a col vector multiply a matrix is the
            // transpose of the vector multiply the matrix.
            real jmj = vc->m_inv_mass_a + vc->m_inv_mass_b +
                       (ra_n * vc->m_inv_I_a).dot(ra_n) +
                       (rb_n * vc->m_inv_I_b).dot(rb_n);

            vcp->m_normal_mass = jmj > 0 ? real(1.0) / jmj : 0;

            real v_rel = vc->m_normal.dot(v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra));
            vcp->m_bias_velocity = 0.0;
            if (v_rel < -vc->m_restitution_threshold) {
                vcp->m_bias_velocity = -vc->m_restitution * v_rel;
            }

            vcp->m_normal_impulse = 0.0;
            vcp->m_normal_contact_impulse = 0.0;
            vcp->m_tangent_impulse = 0.0;
        }
        vc->m_normal_contact_impulse = 0.0;
        vc->m_normal_collision_impulse = 0.0;

        const int32 point_count = vc->m_point_count;

//        if (point_count > 1 && g_block_solve) {
//
//            b3Vec12r* J = (b3Vec12r*)m_block_allocator->allocate(point_count * sizeof(b3Vec12r));
//            b3Vec12r* JW = (b3Vec12r*)m_block_allocator->allocate(point_count * sizeof(b3Vec12r));
//            real* mem_JWJT = (real*)m_block_allocator->allocate(point_count * point_count * sizeof(real));
//            vc->m_JWJT = (real**)m_block_allocator->allocate(point_count * sizeof(real*));
//
//            // calculate JWJT
//            b3Mat1212r W;
//            W.set_zero();
//            W.set_block(vc->m_inv_mass_a * b3Mat33r::identity(), 0, 0);
//            W.set_block(vc->m_inv_I_a, 3, 3);
//            W.set_block(vc->m_inv_mass_b * b3Mat33r::identity(), 6, 6);
//            W.set_block(vc->m_inv_I_b, 9, 9);
//
//            for (int32 i = 0; i < point_count; ++i) {
//                vc->m_JWJT[i] = &mem_JWJT[i * point_count];
//            }
//
//            for (int32 i = 0; i < point_count; i++) {
//                b3VelocityConstraintPoint* vcp = vc->m_points;
//                J[i].set_segment(-vc->m_normal, 0);
//                J[i].set_segment(-vcp[i].m_ra.cross(vc->m_normal), 3);
//                J[i].set_segment(vc->m_normal, 6);
//                J[i].set_segment(vcp[i].m_rb.cross(vc->m_normal), 9);
//            }
//
//            for (int32 i = 0; i < point_count; i++) {
//                JW[i].set_zero();
//                for (int32 j = 0; j < 12; j++) {
//                    JW[i] += J[i][j] * W.row(j);
//                }
//            }
//
//            for (int32 i = 0; i < point_count; ++i) {
//                for (int32 j = 0; j < point_count; ++j) {
//                    vc->m_JWJT[i][j] = JW[i].dot(J[j]);
//                }
//            }
//
//            m_block_allocator->free(J, point_count * sizeof(b3Vec12r));
//            m_block_allocator->free(JW, point_count * sizeof(b3Vec12r));
//        }
    }
}


void b3SolverGR::find_violated_constraints() {

    m_violated_count = 0;

    for (int32 i = 0; i < m_contact_count; ++i) {
        b3ContactVelocityConstraint *vc = m_velocity_constraints + i;
        int32 point_count = vc->m_point_count;
        b3Vec3r v_a = m_vs[vc->m_index_a];
        b3Vec3r w_a = m_ws[vc->m_index_a];
        b3Vec3r v_b = m_vs[vc->m_index_b];
        b3Vec3r w_b = m_ws[vc->m_index_b];
        const b3Vec3r& normal = vc->m_normal;
        for (int32 j = 0; j < point_count; ++j) {
            b3VelocityConstraintPoint* vcp = vc->m_points + j;
            real v_rel = (v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra)).dot(normal);
            if (v_rel < 0) {
                m_violated_constraints[m_violated_count++] = vc;
                for (int32 k = 0; k < point_count; ++k) {
                    b3VelocityConstraintPoint* vcp = vc->m_points + k;
                    real v_rel_in = (v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra)).dot(normal);
                    vcp->m_bias_velocity = - vc->m_restitution * v_rel_in;
                }
                continue;
            }
        }
    }
}


void b3SolverGR::solve_velocity_constraints(int32 velocity_iterations)
{

    find_violated_constraints();

    int32 iteration_max = 50;

    int32 iteration = 0;

    while (m_violated_count != 0 && iteration++ < iteration_max) {

        for (int32 it = 0; it < velocity_iterations; it++) {
            for (int32 i = 0; i < m_violated_count; i++) {
                b3ContactVelocityConstraint* vc = m_violated_constraints[i];
                const b3Vec3r& normal = vc->m_normal;

                b3Vec3r v_a = m_vs[vc->m_index_a];
                b3Vec3r w_a = m_ws[vc->m_index_a];

                b3Vec3r v_b = m_vs[vc->m_index_b];
                b3Vec3r w_b = m_ws[vc->m_index_b];

                for (int32 j = 0; j < vc->m_point_count; ++j) {
                    b3VelocityConstraintPoint* vcp = vc->m_points + j;

                    b3Vec3r v_rel = v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra);

                    real vn = v_rel.dot(normal);

                    real lambda = -vcp->m_normal_mass * (vn - vcp->m_bias_velocity);

                    real new_impulse = b3_max(vcp->m_normal_impulse + lambda, (real)0.0);
                    lambda = new_impulse - vcp->m_normal_impulse;
                    vcp->m_normal_impulse = new_impulse;

                    b3Vec3r impulse = lambda * normal;

                    v_a = v_a - vc->m_inv_mass_a * impulse;
                    w_a = w_a - vc->m_inv_I_a * vcp->m_ra.cross(impulse);
                    v_b = v_b + vc->m_inv_mass_b * impulse;
                    w_b = w_b + vc->m_inv_I_b * vcp->m_rb.cross(impulse);
                }

                m_vs[vc->m_index_a] = v_a;
                m_vs[vc->m_index_b] = v_b;
                m_ws[vc->m_index_a] = w_a;
                m_ws[vc->m_index_b] = w_b;
            }
        }
        find_violated_constraints();
    }
}


b3SolverGR::~b3SolverGR()
{
//    for (int32 i = 0; i < m_contact_count; ++i) {
//        const int32& point_count = m_velocity_constraints[i].m_point_count;
//        if (point_count > 1 && g_block_solve) {
//            m_block_allocator->free(m_velocity_constraints[i].m_JWJT[0], point_count * point_count * sizeof(real));
//            m_block_allocator->free(m_velocity_constraints[i].m_JWJT, point_count * sizeof(real*));
//        }
//    }

    m_block_allocator->free(m_violated_constraints, m_contact_count * sizeof(b3ContactVelocityConstraint*));
    m_block_allocator->free(m_velocity_constraints, m_contact_count * sizeof(b3ContactVelocityConstraint));
    m_block_allocator->free(m_ps, m_body_count * sizeof(b3Vec3r));
    m_block_allocator->free(m_qs, m_body_count * sizeof(b3Quaternionr));
    m_block_allocator->free(m_vs, m_body_count * sizeof(b3Vec3r));
    m_block_allocator->free(m_ws, m_body_count * sizeof(b3Vec3r));
}



