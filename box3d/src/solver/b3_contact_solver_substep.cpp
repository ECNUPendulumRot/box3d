
#include "solver/b3_contact_solver_substep.hpp"

#include "dynamics/b3_island.hpp"
#include "dynamics/b3_body.hpp"


#include "collision/b3_fixture.hpp"
#include "collision/b3_contact.hpp"

#include "common/b3_time_step.hpp"
#include "common/b3_block_allocator.hpp"

#include "solver/b3_contact_constraint.hpp"
#include "solver/b3_lemke.hpp"

#include "spdlog/spdlog.h"
#include "solver/b3_contact_velocity_solver_substep.hpp"


bool g_block_solve = false;

b3ContactSolver::b3ContactSolver(b3BlockAllocator *block_allocator, b3Island *island, b3TimeStep *step, bool is_static)
{
    init(block_allocator, island, step, is_static);
}


void b3ContactSolver::init(b3BlockAllocator *block_allocator, b3Island *island, b3TimeStep *step, bool is_static)
{
    m_is_static = is_static;

    m_timestep = step;
    m_block_allocator = block_allocator;

    m_contact_count = island->get_contacts_count();
    m_contacts = island->get_contacts();

    b3_assert(m_contact_count >= 0);
}


void prepare_contact_sims(b3Island* island, b3ContactSim* css) {
    int32 static_collide = island->m_contact_count;
    for (int32 i = 0; i < static_collide; ++i) {
        b3Contact* c = island->m_contacts[i];
        b3ContactSim* cs = css + i;

        b3Body* body_a = c->m_fixture_a->m_body;
        b3Body* body_b = c->m_fixture_b->m_body;

        cs->body_sim_a = body_a->m_body_sim;
        cs->body_sim_b = body_b->m_body_sim;

        cs->v_a = cs->body_sim_a->v;
        cs->w_a = cs->body_sim_a->w;
        cs->p_a = cs->body_sim_a->p;
        cs->q_a = cs->body_sim_a->q;

        cs->v_b = cs->body_sim_b->v;
        cs->w_b = cs->body_sim_b->w;
        cs->p_b = cs->body_sim_b->p;
        cs->q_b = cs->body_sim_b->q;

        cs->m_a = body_a->m_mass;
        cs->m_b = body_b->m_mass;

        cs->inv_m_a = body_a->m_inv_mass;
        cs->inv_m_b = body_b->m_inv_mass;

        cs->radius_a = c->m_fixture_a->m_shape->m_radius;
        cs->radius_b = c->m_fixture_b->m_shape->m_radius;

        b3Transr xf_a, xf_b;
        xf_a.set_quaternion(cs->q_a);
        xf_b.set_quaternion(cs->q_b);
        xf_a.set_position(cs->p_a);
        xf_b.set_position(cs->p_b);
        cs->world_manifold.initialize(&c->m_manifold, xf_a, cs->radius_a, xf_b, cs->radius_b);

        cs->restitution = c->m_restitution;
        cs->normal = cs->world_manifold.normal;
        cs->point_count = c->m_manifold.m_point_count;

        for (int32 j = 0; j < cs->point_count; ++j) {
            b3VelocityConstraintPoint *vcp = cs->points + j;

            vcp->m_ra = cs->world_manifold.points[j] - cs->p_a;
            vcp->m_rb = cs->world_manifold.points[j] - cs->p_b;

            b3Vec3r rn_a = vcp->m_ra.cross(cs->normal);
            b3Vec3r rn_b = vcp->m_rb.cross(cs->normal);

            // JM_INV_J
            // In box3d, a col vector multiply a matrix is the
            // transpose of the vector multiply the matrix.
            real jmj = cs->inv_m_a + cs->inv_m_b +
                       (rn_a * cs->inv_I_a).dot(rn_a) +
                       (rn_b * cs->inv_I_b).dot(rn_b);

            vcp->m_normal_mass = jmj > 0 ? real(1.0) / jmj : 0;

            // 1. Mv+ = Mv + J^T * lambda ==> Jv+ = Jv + JM_invJ^T * lambda
            // 2. Jv+ = J(-ev)
            // ===> JM_invJ^T * lambda = -eJv - Jv
            real v_rel = cs->normal.dot(cs->v_b + cs->w_b.cross(vcp->m_rb) - cs->v_a - cs->v_b.cross(vcp->m_ra));
            // m_bias_velocity is eJv
            vcp->m_bias_velocity = 0.0;
            if (v_rel < 0) {
                vcp->m_bias_velocity = -cs->restitution * v_rel;
            }

            vcp->m_normal_impulse = 0.0;
        }
    }
}


void b3ContactSolver::prepare_contact_contraints() {

    m_contact_count = m_island->m_contact_count;
    void* mem = m_block_allocator->allocate(m_contact_count * sizeof(b3ContactSim));
    m_contact_constraints = new (mem) b3ContactSim[m_contact_count];
    prepare_contact_sims(m_island, m_contact_constraints);

}


void b3ContactSolver::solve_velocity_constraints() {

    int32 vel_iteration = m_timestep->m_velocity_iterations;

    for (int32 i = 0; i < m_contact_count; ++i) {

        b3ContactSim *cs = m_contact_constraints + i;

        const real& m_a = cs->inv_m_a;
        const real& m_b = cs->inv_m_b;
        const b3Mat33r& I_a = cs->inv_I_a;
        const b3Mat33r& I_b = cs->inv_I_b;

        const b3Vec3r& normal = cs->normal;

        b3Vec3r v_a = cs->body_sim_a->v;
        b3Vec3r w_a = cs->body_sim_a->w;
        b3Vec3r v_b = cs->body_sim_b->v;
        b3Vec3r w_b = cs->body_sim_b->w;

        for (int32 j = 0; j < vel_iteration; ++j) {
            for (int32 k = 0; k < cs->point_count; ++k) {
                b3VelocityConstraintPoint* vcp = cs->points + k;

                b3Vec3r v_rel = (v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra));

                real vn = v_rel.dot(normal);
                real lambda = -vcp->m_normal_mass * (vn - vcp->m_bias_velocity);

                real new_impulse = b3_max(vcp->m_normal_impulse + lambda, (real)0.0);
                lambda = new_impulse - vcp->m_normal_impulse;
                vcp->m_normal_impulse = new_impulse;

                b3Vec3r impulse = lambda * normal;
                v_a = v_a - m_a * impulse;
                w_a = w_a - I_a * vcp->m_ra.cross(impulse);
                v_b = v_b + m_b * impulse;
                w_b = w_b + I_b * vcp->m_rb.cross(impulse);
            }
        }
        cs->body_sim_a->v = v_a;
        cs->body_sim_a->w = w_a;
        cs->body_sim_b->v = v_b;
        cs->body_sim_b->w = w_b;
    }
}


// void b3ContactSolver::write_states_back()
// {
//     for(int32 i = 0; i < m_body_count; ++i) {
//         b3Body* b = m_bodies[i];
//         b->m_sweep.p = m_ps[i];
//         b->m_sweep.q = m_qs[i];
//
//         b->set_linear_velocity(m_vs[i]);
//         b->set_angular_velocity(m_ws[i]);
//         printf("b->m_sweep.p: %f\n", b->m_sweep.p);
//         b->set_position(m_ps[i]);
//         b->set_quaternion(m_qs[i]);
//     }
// }


// int b3ContactSolver::solve(bool allow_sleep)
// {
//     // spdlog::info("|||||||||||||||||| Solve ||||||||||||||||||");
//     int32 substep = 4;
//     real hw = m_timestep->m_hw;
//     real dt_sub = real(1.0)/(substep * hw);
//
//     b3ContactSolverDef def;
//     def.step = *m_timestep;
//     def.is_static_collide = m_is_static;
//     def.vel_iteration = m_timestep->m_velocity_iterations;
//     def.dt_substep = dt_sub;
//     def.contacts = m_contacts;
//     def.count = m_contact_count;
//     def.block_allocator = m_block_allocator;
//
//     // b3ContactVelocitySolverSubstep contact_solver(&def);
//     //
//     // contact_solver.init_velocity_constraints();
//     //
//     // contact_solver.solve_velocity_constraints();
//
//     for (int32 i = 0; i < m_contact_count; ++i) {
//
//         b3ContactSim *cs = m_contact_constraints + i;
//
//         const int32& index_a = cs->m_index_a;
//         const int32& index_b = cs->m_index_b;
//         const real& m_a = cs->inv_m_a;
//         const real& m_b = cs->inv_m_b;
//         const b3Mat33r& I_a = cs->inv_I_a;
//         const b3Mat33r& I_b = cs->inv_I_b;
//         const int32& point_count = cs->points;
//
//         b3Vec3r v_a = vc->m_va;
//         b3Vec3r w_a = vc->m_wa;
//         b3Vec3r v_b = vc->m_vb;
//         b3Vec3r w_b = vc->m_wb;
//         const b3Vec3r& normal = vc->m_normal;
//
//         for (int32 j = 0; j < m_vel_iteration; ++j) {
//             for (int32 k = 0; k< vc->m_point_count; ++k) {
//                 b3VelocityConstraintPoint* vcp = vc->m_points + k;
//
//                 b3Vec3r v_rel = (v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra));
//
//                 real vn = v_rel.dot(normal);
//                 real lambda = -vcp->m_normal_mass * (vn - vcp->m_bias_velocity);
//
//                 real new_impulse = b3_max(vcp->m_normal_impulse + lambda, (real)0.0);
//                 lambda = new_impulse - vcp->m_normal_impulse;
//                 vcp->m_normal_impulse = new_impulse;
//
//                 b3Vec3r impulse = lambda * normal;
//                 v_a = v_a - m_a * impulse;
//                 w_a = w_a - I_a * vcp->m_ra.cross(impulse);
//                 v_b = v_b + m_b * impulse;
//                 w_b = w_b + I_b * vcp->m_rb.cross(impulse);
//             }
//         }
//     }
//
//
//
//     allow_sleep = false;
//     if (allow_sleep) {
//         const float lin_tor_sqr = b3_linear_sleep_tolerance * b3_linear_sleep_tolerance;
//         const float ang_tor_sqr = b3_angular_sleep_tolerance * b3_angular_sleep_tolerance;
//
//         for (int32 i = 0; i < m_body_count; ++i) {
//             b3Body* b = m_bodies[i];
//             if (b->get_type() == b3BodyType::b3_static_body) {
//                 continue;
//             }
//
//             b3Vec3r lin_vel = b->get_linear_velocity();
//             b3Vec3r ang_vel = b->get_angular_velocity();
//
//             if (lin_vel.dot(lin_vel) < lin_tor_sqr && ang_vel.dot(ang_vel) < ang_tor_sqr) {
//                 b->set_awake(false);
//             } else {
//                 b->set_awake(true);
//             }
//         }
//     }
//
//     return 0;
// }


b3ContactSolver::~b3ContactSolver()
{
    ;
}


