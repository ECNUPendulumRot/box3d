
#include <iostream>
#include "solver/b3_contact_solver_jacobi.hpp"

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


b3ContactSolverJacobi::b3ContactSolverJacobi(b3BlockAllocator *block_allocator, b3Island *island, b3TimeStep *step)
{
    init(block_allocator, island, step);
}


void b3ContactSolverJacobi::init(b3BlockAllocator *block_allocator, b3Island *island, b3TimeStep *step)
{
    m_timestep = step;
    m_block_allocator = block_allocator;
    m_island = island;
    m_contact_count = island->get_contacts_count();
    m_contacts = island->get_contacts();

    b3_assert(m_contact_count >= 0);
}


static void prepare_contact_sims(b3Island* island, b3ContactSimJacobi* css) {
    int32 collide = island->m_contact_count;
    for (int32 i = 0; i < collide; ++i) {
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
            real v_rel = cs->normal.dot(cs->v_b + cs->w_b.cross(vcp->m_rb) - cs->v_a - cs->w_b.cross(vcp->m_ra));
            // m_bias_velocity is eJv
            vcp->m_bias_velocity = 0.0;
            if (v_rel < 0) {
                vcp->m_bias_velocity = -cs->restitution * v_rel;
            }
            vcp->m_iter_bias_velocity = vcp->m_bias_velocity;
            vcp->m_normal_impulse = 0.0;
        }
    }
}


void b3ContactSolverJacobi::prepare_contact_contraints() {

    m_contact_count = m_island->m_contact_count;
    void* mem = m_block_allocator->allocate(m_contact_count * sizeof(b3ContactSimJacobi));
    m_contact_constraints = new (mem) b3ContactSimJacobi[m_contact_count];

    prepare_contact_sims(m_island, m_contact_constraints);

}


void b3ContactSolverJacobi::solve_velocity_constraints() {

    int32 vel_iteration = m_timestep->m_velocity_iterations;

    for (int32 i = 0; i < vel_iteration; ++i) {

        for (int32 j = 0; j < m_contact_count; ++j) {

            b3ContactSimJacobi *cs = m_contact_constraints + j;

            const real& m_a = cs->inv_m_a;
            const real& m_b = cs->inv_m_b;
            const b3Mat33r& I_a = cs->inv_I_a;
            const b3Mat33r& I_b = cs->inv_I_b;

            const b3Vec3r& normal = cs->normal;

            b3Vec3r v_a = cs->body_sim_a->v;
            b3Vec3r w_a = cs->body_sim_a->w;
            b3Vec3r v_b = cs->body_sim_b->v;
            b3Vec3r w_b = cs->body_sim_b->w;

            for (int32 k = 0; k < cs->point_count; ++k) {
                b3VelocityConstraintPoint* vcp = cs->points + k;

                b3Vec3r v_rel = (v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra));

                real vn = v_rel.dot(normal);
                real lambda = -vcp->m_normal_mass * (vn - vcp->m_iter_bias_velocity);
//                if(i<4) {
//                    std::cout << "vn: " << vn << std::endl;
//                    std::cout << "m_bias_velocity: " << vcp->m_bias_velocity << std::endl;
//                }
                // real new_impulse = b3_max(vcp->m_normal_impulse + lambda, (real)0.0);
                real new_impulse = b3_max(lambda, (real)0.0);
                // lambda = new_impulse - vcp->m_normal_impulse;
                lambda = new_impulse;
                vcp->m_normal_impulse = new_impulse;

                b3Vec3r impulse = lambda * normal;

                cs->delta_v_a = -m_a * impulse;
                cs->delta_w_a = -(I_a * vcp->m_ra.cross(impulse));
                cs->delta_v_b = m_b * impulse;
                cs->delta_w_b = I_b * vcp->m_rb.cross(impulse);

//                if(i<4){
//                    std::cout <<"lambda_new: "<<lambda<<std::endl;
//                    std::cout <<"impulse: ("<< impulse.x<<","<<impulse.y<<","<<impulse.z<<")"<<std::endl;
//                    std::cout<<std::endl;
//                }
            }
        }

        for (int32 i = 0 ; i < m_contact_count; ++i) {
            b3ContactSimJacobi* cs = m_contact_constraints + i;

            b3BodySim* body_sim_a = cs->body_sim_a;
            b3BodySim* body_sim_b = cs->body_sim_b;

            body_sim_a->v += cs->delta_v_a;
            body_sim_a->w += cs->delta_w_a;
            body_sim_b->v += cs->delta_v_b;
            body_sim_b->w += cs->delta_w_b;
        }

        for (int32 i = 0 ; i < m_contact_count; ++i) {
            b3ContactSimJacobi* cs = m_contact_constraints + i;

            b3BodySim* body_sim_a = cs->body_sim_a;
            b3BodySim* body_sim_b = cs->body_sim_b;

            b3Vec3r v_a = body_sim_a->v;
            b3Vec3r w_a = body_sim_a->w;
            b3Vec3r v_b = body_sim_b->v;
            b3Vec3r w_b = body_sim_b->w;

            for (int32 j = 0; j < cs->point_count; ++j) {
                b3VelocityConstraintPoint* vcp = cs->points + j;
                real v_rel = cs->normal.dot(v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra));
                vcp->m_iter_bias_velocity = 0.0;
                if (v_rel < 0) {
                    vcp->m_iter_bias_velocity = -cs->restitution * v_rel;
                }
            }
        }
    }
}


b3ContactSolverJacobi::~b3ContactSolverJacobi()
{
    m_block_allocator->free(m_contact_constraints, m_contact_count * sizeof(b3ContactSim));
}


