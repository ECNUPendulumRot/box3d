
#include "solver/b3_si_solver.hpp"

#include "dynamics/b3_island.hpp"
#include "dynamics/b3_body.hpp"

#include "collision/b3_fixture.hpp"
#include "collision/b3_collision.hpp"
#include "collision/b3_contact.hpp"

#include "common/b3_time_step.hpp"

#include <iostream>

b3SISolver::b3SISolver(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step) :
                            b3Solver(block_allocator, island, step) {

    for(int32 i = 0; i < m_contact_count; ++i) {
        b3Contact* contact = m_contacts[i];

        b3Fixture* fixture_a = contact->get_fixture_a();
        b3Fixture* fixture_b = contact->get_fixture_b();

        b3Body* body_a = fixture_a->get_body();
        b3Body* body_b = fixture_b->get_body();
        b3Manifold* manifold = contact->get_manifold();

        int32 point_count = manifold->m_point_count;

        b3_assert(point_count >= 0);

        b3ContactVelocityConstraint* vc = m_velocity_constraints + i;
        vc->m_restitution = contact->get_restitution();
        vc->m_contact_index = i;
        vc->m_point_count = point_count;

        vc->m_normal = manifold->m_local_normal;

        vc->m_index_a = body_a->get_island_index();
        vc->m_inv_mass_a = body_a->get_inv_mass();
        vc->m_inv_I_a = body_a->get_inv_inertia();

        vc->m_index_b = body_b->get_island_index();
        vc->m_inv_mass_b = body_b->get_inv_mass();
        vc->m_inv_I_b = body_b->get_inv_inertia();

        vc->m_penetration = manifold->m_penetration;

        // the center of body in the world frame
        b3Vector3d center_a = body_a->get_pose().transform(body_a->get_local_center());
        b3Vector3d center_b = body_b->get_pose().transform(body_b->get_local_center());

        for(int32 j = 0; j < point_count; j++) {
            b3VelocityConstraintPoint* vcp = vc->m_points + j;
            b3ManifoldPoint* manifold_point = manifold->m_points + j;

            vcp->m_ra = manifold_point->m_local_point - center_a;
            vcp->m_rb = manifold_point->m_local_point - center_b;
            vcp->m_rhs_penetration = manifold->m_penetration;

            // TODO: warm start
        }
    }
}


void b3SISolver::init_velocity_constraints() {
    for(int32 i = 0; i < m_contact_count; ++i) {
        b3ContactVelocityConstraint* vc = m_velocity_constraints + i;

        int32 index_a = vc->m_index_a;
        int32 index_b = vc->m_index_b;

        b3Vector3d v_a = m_velocities[index_a].linear();
        b3Vector3d w_a = m_velocities[index_a].angular();

        b3Vector3d v_b = m_velocities[index_b].linear();
        b3Vector3d w_b = m_velocities[index_b].angular();

        for(int j = 0; j < vc->m_point_count; ++j) {
            b3VelocityConstraintPoint* vcp = vc->m_points + j;

            b3Vector3d ra_n = vcp->m_ra.cross(vc->m_normal);
            b3Vector3d rb_n = vcp->m_rb.cross(vc->m_normal);

            // JM_INV_J
            // In box3d, a col vector multiply a matrix is the transpose of the vector multiply the matrix.
            double jmj = vc->m_inv_mass_a + vc->m_inv_mass_b +
                        (ra_n * vc->m_inv_I_a).dot(ra_n) +
                        (rb_n * vc->m_inv_I_b).dot(rb_n);

            vcp->m_normal_mass = jmj > 0 ? 1.0 / jmj : 0;

            // TODO：tangent friction plane
            

            // 1. Mv+ = Mv + Jlambda ==> Jv+ = Jv + JM_invJlambda
            // 2. Jv+ = -penetration / dt + J(-ev)
            // ===> JM_invJlambda = -penetration / dt - eJv - Jv
            double v_rel = vc->m_normal.dot(v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra));
            vcp->m_rhs_restitution_velocity = -vc->m_restitution * v_rel;
            vcp->m_rhs_penetration = vcp->m_rhs_penetration * m_timestep->m_inv_dt;

            vcp->m_normal_impulse = 0.0;
            vcp->m_tangent_impulse = 0.0;
        }

        // TODO: if we have more than one contact point, then prepare the block solver.
    }
}


int b3SISolver::solve() {

    init_velocity_constraints();

    for(int32 i = 0; i < m_timestep->m_velocity_iterations; ++i) {
        solve_velocity_constraints();
    }

    // integrate position
    for(int32 i = 0; i < m_body_count; ++i) {
        m_positions[i].set_linear(m_positions[i].linear() + m_velocities[i].linear() * m_timestep->m_dt);
        m_positions[i].set_angular(m_positions[i].angular() + m_velocities[i].angular() * m_timestep->m_dt);
    }

    // copy state buffers back to the bodies.
    write_states_back();

    return 0;
}


void b3SISolver::solve_velocity_constraints() {
    for(int32 i = 0; i < m_contact_count; ++i) {
        b3ContactVelocityConstraint* vc = m_velocity_constraints + i;

        int32 point_count = vc->m_point_count;

        b3Vector3d v_a = m_velocities[vc->m_index_a].linear();
        b3Vector3d w_a = m_velocities[vc->m_index_a].angular();

        b3Vector3d v_b = m_velocities[vc->m_index_b].linear();
        b3Vector3d w_b = m_velocities[vc->m_index_b].angular();

        if (point_count == 1) {
            for (int32 j = 0; j < vc->m_point_count; ++j) {
                b3VelocityConstraintPoint* vcp = vc->m_points + j;

                b3Vector3d v_rel = v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra);
                double rhs = -v_rel.dot(vc->m_normal);
                // double lambda = vcp->m_normal_mass * (rhs + vcp->m_rhs_restitution_velocity + vcp->m_rhs_penetration);
                double lambda = vcp->m_normal_mass * (rhs + vcp->m_rhs_restitution_velocity);

                double new_impluse = b3_max(vcp->m_normal_impulse + lambda, 0.0);
                lambda = new_impluse - vcp->m_normal_impulse;
                vcp->m_normal_impulse = new_impluse;

                // apply normal Impluse
                b3Vector3d impluse = lambda * vc->m_normal;

                v_a = v_a - vc->m_inv_mass_a * impluse;
                w_a = w_a - vc->m_inv_I_a * vcp->m_ra.cross(impluse);
                v_b = v_b + vc->m_inv_mass_b * impluse;
                w_b = w_b + vc->m_inv_I_b * vcp->m_rb.cross(impluse);
            }
        }

        m_velocities[vc->m_index_a].set_linear(v_a);
        m_velocities[vc->m_index_a].set_angular(w_a);
        m_velocities[vc->m_index_b].set_linear(v_b);
        m_velocities[vc->m_index_b].set_angular(w_b);

        // TODO: Check this way is useful. And angle ?
//        if(vc->m_penetration < 0) {
//            b3Vector3d p_a = m_positions[vc->m_index_a].linear();
//            b3Vector3d p_b = m_positions[vc->m_index_b].linear();
//
//            p_a += vc->m_normal * vc->m_penetration;
//            p_b -= vc->m_normal * vc->m_penetration;
//
//            m_positions[vc->m_index_a].set_linear(p_a);
//            m_positions[vc->m_index_b].set_linear(p_b);
//
//            vc->m_penetration = 0;
//        }
    }
}