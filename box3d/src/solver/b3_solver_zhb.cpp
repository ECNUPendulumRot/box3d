
#include "solver/b3_solver_zhb.hpp"

#include "dynamics/b3_island.hpp"
#include "dynamics/b3_body.hpp"
#include "collision/b3_collision.hpp"
#include "collision/b3_contact.hpp"
#include "common/b3_time_step.hpp"
#include "common/b3_block_allocator.hpp"
#include "collision/b3_fixture.hpp"


b3SolverZHB::b3SolverZHB(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step) {
    init(block_allocator, island, step);
}


void b3SolverZHB::init(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step) {
    m_method = step->m_integral_method;
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
    if (m_contact_count > 0) {
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

    for (int32 i = 0; i < m_body_count; ++i) {
        b3Body* b = m_bodies[i];
        m_ps[i] = b->get_position();
        m_qs[i] = b->get_quaternion();
        m_vs[i] = b->get_linear_velocity();
        m_ws[i] = b->get_angular_velocity();
    }

    ////////////////////////////////////////////////////////////////////////////////

    for (int32 i = 0; i < m_contact_count; ++i) {
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

        vc->m_normal = manifold->m_local_normal;

        vc->m_index_a = body_a->get_island_index();
        vc->m_mass_a = body_a->get_mass();
        vc->m_inv_mass_a = body_a->get_inv_mass();

        const b3Mat33r& R_a = body_a->get_quaternion().rotation_matrix();

        vc->m_I_a = R_a * body_a->get_inertia() * R_a.transpose();
        vc->m_inv_I_a = R_a * body_a->get_inv_inertia() * R_a.transpose();

        vc->m_index_b = body_b->get_island_index();

        vc->m_mass_b = body_b->get_mass();
        vc->m_inv_mass_b = body_b->get_inv_mass();

        const b3Mat33r& R_b = body_b->get_quaternion().rotation_matrix();

        vc->m_I_b = R_b * body_b->get_inertia() * R_b.transpose();
        vc->m_inv_I_b = R_b * body_b->get_inv_inertia() * R_b.transpose();

        vc->m_penetration = manifold->m_penetration;

        vc->m_ra = b3Vec3r::zero();
        vc->m_rb = b3Vec3r::zero();
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
            // vcp->m_rhs_penetration = manifold->m_penetration;
            // TODO: warm start
        }
    }

}


void b3SolverZHB::write_states_back() {
    for (int32 i = 0; i < m_body_count; ++i) {
        m_bodies[i]->set_position(m_ps[i]);
        m_bodies[i]->set_quaternion(m_qs[i]);
        m_bodies[i]->set_linear_velocity(m_vs[i]);
        m_bodies[i]->set_angular_velocity(m_ws[i]);
        // TODO: if we need SynchronizeTransform() ?
    }
}


b3SolverZHB::~b3SolverZHB() {
    clear();
}


void b3SolverZHB::clear() {
    m_timestep = nullptr;
    m_contacts = nullptr;

    m_block_allocator->free(m_velocity_constraints, m_contact_count * sizeof(b3ContactVelocityConstraint));
    m_block_allocator->free(m_ps, m_body_count * sizeof(b3Vec3r));
    m_block_allocator->free(m_qs, m_body_count * sizeof(b3Quaternionr));
    m_block_allocator->free(m_vs, m_body_count * sizeof(b3Vec3r));
    m_block_allocator->free(m_ws, m_body_count * sizeof(b3Vec3r));

    m_block_allocator = nullptr;
}

int b3SolverZHB::solve() {
    init_velocity_constraints();

    // collision
    for (int32 i = 0; i < m_timestep->m_velocity_iterations; ++i) {
        solve_velocity_constraints(true);
    }

    // velocity update
    for (int32 i = 0; i < m_body_count; ++i) {
        b3Body* b = m_bodies[i];
        b3Vec3r v = m_vs[i];
        b3Vec3r w = m_ws[i];

        v += m_timestep->m_dt * b->get_inv_mass() * (b->get_force() + b->get_gravity());
        w += m_timestep->m_dt * b->get_inv_inertia() * b->get_torque();

        m_vs[i] = v;
        m_ws[i] = w;
    }

    for (int32 i = 0; i < m_timestep->m_velocity_iterations; ++i) {
        solve_velocity_constraints(false);
    }

    for (int32 i = 0; i < m_body_count; ++i) {
        m_ps[i] = m_ps[i] + m_vs[i] * m_timestep->m_dt;
        m_qs[i] = m_qs[i] + real(0.5) * m_timestep->m_dt * b3Quaternionr(0, m_ws[i]) * m_qs[i];
        m_qs[i].normalize();
    }

    // copy state buffers back to the bodies.
    write_states_back();

    return 0;
}


void b3SolverZHB::solve_velocity_constraints(bool is_collision) {
    real tolerance = 0.01f;
    for (int32 i = 0; i < m_contact_count; ++i) {
        b3ContactVelocityConstraint* vc = m_velocity_constraints + i;

        b3Vec3r v_a = m_vs[vc->m_index_a];
        b3Vec3r w_a = m_ws[vc->m_index_a];

        b3Vec3r v_b = m_vs[vc->m_index_b];
        b3Vec3r w_b = m_ws[vc->m_index_b];

        if (true) {

            for (int32 j = 0; j < vc->m_point_count; ++j) {
                b3VelocityConstraintPoint* vcp = vc->m_points + j;

                b3Vec3r v_rel = v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra);
                real rhs = -v_rel.dot(vc->m_normal);

                real rhs_restitution_velocity = vcp->m_rhs_restitution_velocity;

                //there are four situations of constrains
                //1.init constrain violate,now legal
                //2.init constrain violate,now still violate
                //3.init constrain legal,now legal
                //4.init constrain legal,now violate
                if (rhs > 0) {
                    if (rhs_restitution_velocity <= 0) {
                        //4
                        if (abs(vcp->m_relative_velocity - rhs) <= tolerance) {
                            //if rhs is converged,change it to situation 2
                            vcp->m_rhs_restitution_velocity = rhs * vc->m_restitution;
                            vcp->m_relative_velocity = rhs;
                            rhs_restitution_velocity = 0;
                            rhs = 0.0;
                        } else {
                            //have some problem
                            vcp->m_relative_velocity = rhs;
                            rhs = 0.0;
                            rhs_restitution_velocity = 0.0;
                        }
                    } else {
                        //2

                    }
                }

                //update the -eJv
                if (rhs <= 0) {
                    if (rhs_restitution_velocity > 0) {
                        //1
                        if (abs(vcp->m_relative_velocity - rhs) <= tolerance || rhs == 0.0f) {
                            rhs = 0.0;
                            rhs_restitution_velocity = 0.0;
                            vcp->m_relative_velocity = 0.0f;
                            //change to situation 3
                            vcp->m_rhs_restitution_velocity = 0.0f;
                        }
                    } else {
                        //3
                        rhs = 0.0;
                        rhs_restitution_velocity = 0.0;
                    }
                }

                real lambda = 0;

                // TODO:
                if (is_collision) {
                    lambda = vcp->m_normal_mass * (rhs + rhs_restitution_velocity);
                    real new_impulse = b3_max(vcp->m_normal_collision_impulse + lambda, (real)0.0);
                    lambda = new_impulse - vcp->m_normal_collision_impulse;
                    vcp->m_normal_collision_impulse = new_impulse;
                } else {
                    lambda = vcp->m_normal_mass * rhs;
                    real new_impulse = b3_max(vcp->m_normal_contact_impulse + lambda, (real)0.0);
                    lambda = new_impulse - vcp->m_normal_contact_impulse;
                    vcp->m_normal_contact_impulse = new_impulse;
                }

                // apply normal Impulse
                b3Vec3r impulse = lambda * vc->m_normal;

                v_a = v_a - vc->m_inv_mass_a * impulse;
                w_a = w_a - vc->m_inv_I_a * vcp->m_ra.cross(impulse);
                v_b = v_b + vc->m_inv_mass_b * impulse;
                w_b = w_b + vc->m_inv_I_b * vcp->m_rb.cross(impulse);
                if (rhs != 0.0f) {
                    //delete these seems have no effect to solver?why?
                    v_rel = v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra);
                    vcp->m_relative_velocity = -v_rel.dot(vc->m_normal);
                }
            }

        } else {
            // vn = A * x + b
            // A = J * M_inv * JT
            // b = vn0 - v_bias

        }


        m_vs[vc->m_index_a] = v_a;
        m_vs[vc->m_index_b] = v_b;
        m_ws[vc->m_index_a] = w_a;
        m_ws[vc->m_index_b] = w_b;
    }
}


void b3SolverZHB::init_velocity_constraints() {
    for (int32 i = 0; i < m_contact_count; ++i) {
        b3ContactVelocityConstraint* vc = m_velocity_constraints + i;
        int32 index_a = vc->m_index_a;
        int32 index_b = vc->m_index_b;

        b3Vec3r v_a = m_vs[index_a];
        b3Vec3r w_a = m_ws[index_a];

        b3Vec3r v_b = m_vs[index_b];
        b3Vec3r w_b = m_ws[index_b];

        for (int j = 0; j < vc->m_point_count; ++j) {
            b3VelocityConstraintPoint* vcp = vc->m_points + j;

            // vcp->m_ra(b) is a contact point(from the center of body)
            // vc->m_ra(b) is the average of all contact points
            vc->m_ra += vcp->m_ra;
            vc->m_rb += vcp->m_rb;

            b3Vec3r ra_n = vcp->m_ra.cross(vc->m_normal);
            b3Vec3r rb_n = vcp->m_rb.cross(vc->m_normal);

            // JM_INV_J
            // In box3d, a col vector multiply a matrix is the
            // transpose of the vector multiply the matrix.
            real jmj = vc->m_inv_mass_a + vc->m_inv_mass_b +
                (ra_n * vc->m_inv_I_a).dot(ra_n) +
                (rb_n * vc->m_inv_I_b).dot(rb_n);

            vcp->m_normal_mass = jmj > 0 ? real(1.0) / jmj : 0;

            // 1. Mv+ = Mv + J^T * lambda ==> Jv+ = Jv + JM_invJ^T * lambda
            // 2. Jv+ = J(-ev)
            // ===> JM_invJ^T * lambda = -eJv - Jv
            real v_rel = vc->m_normal.dot(v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra));
            // m_rhs_restitution_velocity is eJv
            vcp->m_rhs_restitution_velocity = -vc->m_restitution * v_rel;

            vcp->m_normal_collision_impulse = 0.0;
            vcp->m_normal_contact_impulse = 0.0;
            vcp->m_tangent_impulse = 0.0;
        }
        vc->m_normal_contact_impulse = 0.0;
        vc->m_normal_collision_impulse = 0.0;
        vc->m_ra /= vc->m_point_count;
        vc->m_rb /= vc->m_point_count;

        // TODO: if we have more than one contact point, then prepare the block solver.
    }
}


