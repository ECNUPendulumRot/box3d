
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

        memory = m_block_allocator->allocate(m_contact_count * sizeof(b3ContactPositionConstraint));
        m_position_constraints = new (memory) b3ContactPositionConstraint;

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

    memory = m_block_allocator->allocate(m_body_count * sizeof(b3Quatr));
    m_qs = new (memory) b3Quatr;

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
        b3Shape* shape_a = fixture_a->get_shape();
        b3Shape* shape_b = fixture_b->get_shape();
        b3Body* body_a = fixture_a->get_body();
        b3Body* body_b = fixture_b->get_body();
        real radius_a = shape_a->get_radius();
        real radius_b = shape_b->get_radius();
        b3Manifold* manifold = contact->get_manifold();

        int32 point_count = manifold->m_point_count;
        b3_assert(point_count > 0);

        //////////////////////////// Velocity Constraints ////////////////////////////
        b3ContactVelocityConstraint* vc = m_velocity_constraints + i;
        vc->m_friction = contact->get_friction();
        vc->m_restitution = contact->get_restitution();
        vc->m_restitution_threshold = contact->get_restitution_threshold();

        vc->m_index_a = body_a->get_island_index();
        vc->m_index_b = body_b->get_island_index();
        vc->m_inv_mass_a = body_a->get_inv_mass();
        vc->m_inv_mass_b = body_b->get_inv_mass();

        const b3Mat33r& R_a = body_a->get_quaternion().rotation_matrix();
        vc->m_inv_I_a = R_a.transpose() * body_a->get_inv_inertia() * R_a;
        const b3Mat33r& R_b = body_b->get_quaternion().rotation_matrix();
        vc->m_inv_I_b = R_b.transpose() * body_b->get_inv_inertia();

        vc->m_point_count = point_count;
        vc->m_contact_index = i;

        //////////////////////////// Position Constraints ////////////////////////////
        b3ContactPositionConstraint* pc = m_position_constraints + i;
        pc->m_index_a = vc->m_index_a;
        pc->m_index_b = vc->m_index_b;
        pc->m_inv_mass_a = vc->m_inv_mass_a;
        pc->m_inv_mass_b = vc->m_inv_mass_b;
        pc->m_local_center_a = body_a->get_local_center();
        pc->m_local_center_b = body_b->get_local_center();
        pc->m_inv_I_a = vc->m_inv_I_a;
        pc->m_inv_I_b = vc->m_inv_I_b;
        pc->m_local_normal = manifold->m_local_normal;
        pc->m_local_point = manifold->m_local_point;
        pc->m_point_count = point_count;
        pc->m_radius_a = radius_a;
        pc->m_radius_b = radius_b;
        pc->m_type = manifold->m_type;

        for (int32 j = 0; j < point_count; j++) {

            b3ManifoldPoint* manifold_point = manifold->m_points + j;
            b3VelocityConstraintPoint* vcp = vc->m_points + j;
            vcp->m_ra.set_zero();
            vcp->m_rb.set_zero();

            vcp->m_normal_mass = 0.0;
            vcp->m_bias_velocity = 0.0;

            pc->m_local_points[j] = manifold_point->m_local_point;
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
        m_qs[i] = m_qs[i] + real(0.5) * m_timestep->m_dt * b3Quatr(0, m_ws[i]) * m_qs[i];
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
        b3ContactPositionConstraint* pc = m_position_constraints + i;

        real radius_a = pc->m_radius_a;
        real radius_b = pc->m_radius_b;
        b3Manifold* manifold = m_contacts[vc->m_contact_index]->get_manifold();

        int32 index_a = vc->m_index_a;
        int32 index_b = vc->m_index_b;

        const real& m_a = vc->m_inv_mass_a;
        const real& m_b = vc->m_inv_mass_b;
        const b3Mat33r& I_a = vc->m_inv_I_a;
        const b3Mat33r& I_b = vc->m_inv_I_b;
        const b3Vec3r& local_center_a = pc->m_local_center_a;
        const b3Vec3r& local_center_b = pc->m_local_center_b;

        b3Vec3r p_a = m_ps[index_a];
        b3Quatr q_a = m_qs[index_a];
        b3Vec3r v_a = m_vs[index_a];
        b3Vec3r w_a = m_ws[index_a];

        b3Vec3r p_b = m_ps[index_b];
        b3Quatr q_b = m_qs[index_b];
        b3Vec3r v_b = m_vs[index_b];
        b3Vec3r w_b = m_ws[index_b];

        b3Transr xf_a, xf_b;
        xf_a.set_quaternion(q_a);
        xf_b.set_quaternion(q_b);
        xf_a.set_position(p_a - xf_a.rotate(local_center_a));
        xf_b.set_position(p_b - xf_b.rotate(local_center_b));

        b3WorldManifold world_manifold;
        world_manifold.initialize(manifold, xf_a, radius_a, xf_b, radius_b);

        vc->m_normal = world_manifold.normal;

        int32 point_count = vc->m_point_count;
        for (int32 j = 0; j < point_count; ++j) {
            b3VelocityConstraintPoint *vcp = vc->m_points + j;

            vcp->m_ra = world_manifold.points[j] - p_a;
            vcp->m_rb = world_manifold.points[j] - p_b;

            b3Vec3r rn_a = vcp->m_ra.cross(vc->m_normal);
            b3Vec3r rn_b = vcp->m_rb.cross(vc->m_normal);

            // JM_INV_J
            // In box3d, a col vector multiply a matrix is the
            // transpose of the vector multiply the matrix.
            real jmj = m_a + m_b +
                       (rn_a * I_a).dot(rn_a) +
                       (rn_b * I_b).dot(rn_b);

            vcp->m_normal_mass = jmj > 0 ? real(1.0) / jmj : 0;

            // 1. Mv+ = Mv + J^T * lambda ==> Jv+ = Jv + JM_invJ^T * lambda
            // 2. Jv+ = J(-ev)
            // ===> JM_invJ^T * lambda = -eJv - Jv
            real v_rel = vc->m_normal.dot(v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra));
            // m_bias_velocity is eJv
            vcp->m_bias_velocity = 0.0;
            if (v_rel < -vc->m_restitution_threshold) {
                vcp->m_bias_velocity = -vc->m_restitution * v_rel;
            }

            vcp->m_normal_impulse = 0.0;
        }
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
                break;
            }
        }
    }
}


void b3SolverGR::solve_velocity_constraints(int32 velocity_iterations)
{

    find_violated_constraints();

   // int32 iteration_max = 50;

    int32 iteration = 1;

    while (m_violated_count != 0 && iteration++) {
        spdlog::info("Iteration: {0}", iteration);
        spdlog::info("Violated count: {0}", m_violated_count);
        for (int32 i = 0; i < m_violated_count; i++) {
            b3ContactVelocityConstraint* vc = m_violated_constraints[i];
            spdlog::info("violated pair: {}, {}", vc->m_index_a, vc->m_index_b);
        }
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
    m_block_allocator->free(m_violated_constraints, m_contact_count * sizeof(b3ContactVelocityConstraint*));
    m_block_allocator->free(m_velocity_constraints, m_contact_count * sizeof(b3ContactVelocityConstraint));
    m_block_allocator->free(m_position_constraints, m_contact_count * sizeof(b3ContactPositionConstraint*));
    m_block_allocator->free(m_ps, m_body_count * sizeof(b3Vec3r));
    m_block_allocator->free(m_qs, m_body_count * sizeof(b3Quatr));
    m_block_allocator->free(m_vs, m_body_count * sizeof(b3Vec3r));
    m_block_allocator->free(m_ws, m_body_count * sizeof(b3Vec3r));
}



