
#include "solver/b3_velocity_solver.hpp"

#include "dynamics/b3_island.hpp"
#include "dynamics/b3_body.hpp"

#include "collision/b3_fixture.hpp"
#include "collision/b3_collision.hpp"
#include "collision/b3_contact.hpp"

#include "common/b3_time_step.hpp"

#include <spdlog/spdlog.h>


void b3VelocitySolver::init(b3BlockAllocator *block_allocator, b3Island *island, b3TimeStep *step) {

    b3Solver::init(block_allocator, island, step);

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
        vc->m_contact_index = i;
        vc->m_point_count = point_count;

        vc->m_normal = manifold->m_local_normal;

        vc->m_index_a = body_a->get_island_index();
        vc->m_mass_a = body_a->get_mass();
        vc->m_inv_mass_a = body_a->get_inv_mass();

        const b3Matrix3r& R_a = body_a->get_quaternion().rotation_matrix();

        vc->m_I_a = R_a * body_a->get_inertia() * R_a.transpose();
        vc->m_inv_I_a = R_a * body_a->get_inv_inertia() * R_a.transpose();

        vc->m_index_b = body_b->get_island_index();

        vc->m_mass_b = body_b->get_mass();
        vc->m_inv_mass_b = body_b->get_inv_mass();

        const b3Matrix3r& R_b = body_b->get_quaternion().rotation_matrix();

        vc->m_I_b = R_b * body_b->get_inertia() * R_b.transpose();
        vc->m_inv_I_b = R_b * body_b->get_inv_inertia() * R_b.transpose();

        vc->m_penetration = manifold->m_penetration;

        vc->m_ra = b3Vector3r::zero();
        vc->m_rb = b3Vector3r::zero();
        // the center of body in the world frame

        b3Transformr xf_a(body_a->get_position(), body_a->get_quaternion());
        b3Transformr xf_b(body_b->get_position(), body_b->get_quaternion());

        b3Vector3r center_a = xf_a.transform(body_a->get_local_center());
        b3Vector3r center_b = xf_b.transform(body_b->get_local_center());

        for (int32 j = 0; j < point_count; j++) {
            b3VelocityConstraintPoint *vcp = vc->m_points + j;
            b3ManifoldPoint *manifold_point = manifold->m_points + j;

            vcp->m_ra = manifold_point->m_local_point - center_a;
            vcp->m_rb = manifold_point->m_local_point - center_b;
            // vcp->m_rhs_penetration = manifold->m_penetration;
            // TODO: warm start
        }
    }
}


void b3VelocitySolver::init_velocity_constraints()
{
    for(int32 i = 0; i < m_contact_count; ++i) {
        b3ContactVelocityConstraint* vc = m_velocity_constraints + i;
        int32 index_a = vc->m_index_a;
        int32 index_b = vc->m_index_b;

        b3Vector3r v_a = m_vs[index_a];
        b3Vector3r w_a = m_ws[index_a];

        b3Vector3r v_b = m_vs[index_b];
        b3Vector3r w_b = m_ws[index_b];

        for (int j = 0; j < vc->m_point_count; ++j) {
            b3VelocityConstraintPoint *vcp = vc->m_points + j;

            // vcp->m_ra(b) is a contact point(from the center of body)
            // vc->m_ra(b) is the average of all contact points
            vc->m_ra += vcp->m_ra;
            vc->m_rb += vcp->m_rb;

            b3Vector3r ra_n = vcp->m_ra.cross(vc->m_normal);
            b3Vector3r rb_n = vcp->m_rb.cross(vc->m_normal);

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


int b3VelocitySolver::solve()
{
    init_velocity_constraints();

    // collision
    for(int32 i = 0; i < m_timestep->m_velocity_iterations; ++i) {
        solve_velocity_constraints(true);
    }
    // fix penetration
    // correct_penetration();

    // velocity update
    for(int32 i = 0; i < m_body_count; ++i) {
        b3Body *b = m_bodies[i];
        b3Vector3r v = m_vs[i];
        b3Vector3r w = m_ws[i];

        v += m_timestep->m_dt * b->get_inv_mass() * (b->get_force() + b->get_gravity());
        w += m_timestep->m_dt * b->get_inv_inertia() * b->get_torque();

        m_vs[i] = v;
        m_ws[i] = w;
    }


    // contact
    for (int32 i = 0; i < m_timestep->m_velocity_iterations; ++i) {
        solve_velocity_constraints(false);
    }

    // solve friction constraints
    // TODO: It needs to be re-implemented
    // solve_friction_constraints();

    // integrate position
    // type 0 = semi-implict Euler integration
    // type 1 = verlet integration

    for (int32 i = 0; i < m_body_count; ++i) {
        m_ps[i] = m_ps[i] + m_vs[i] * m_timestep->m_dt;
        m_qs[i] = m_qs[i] + real(0.5) * m_timestep->m_dt * b3Quaternionr(0, m_ws[i]) * m_qs[i];
        m_qs[i].normalize();
    }

    // copy state buffers back to the bodies.
    write_states_back();

    return 0;
}


void b3VelocitySolver::solve_velocity_constraints(bool is_collision)
{
    for (int32 i = 0; i < m_contact_count; ++i) {
        b3ContactVelocityConstraint *vc = m_velocity_constraints + i;

        b3Vector3r v_a = m_vs[vc->m_index_a];
        b3Vector3r w_a = m_ws[vc->m_index_a];

        b3Vector3r v_b = m_vs[vc->m_index_b];
        b3Vector3r w_b = m_ws[vc->m_index_b];

/*////////////////////////////////////////////////////////////////////////////////////////////////
    real lambda = 0;
    // iter all contact points, compute the total impulse for body
    // TODO: prove this algorithm is correct.
    for (int32 j = 0; j < vc->m_point_count; ++j) {
      b3VelocityConstraintPoint* vcp = vc->m_points + j;

      b3Vector3r v_rel = v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra);
      real rhs = -v_rel.dot(vc->m_normal);
      // distinguish contact and collision
      // collision: we need update velocity by restitution
      // contact: the restitution is zero.
      if(is_collision) {
        lambda = vcp->m_normal_mass * (rhs + vcp->m_rhs_restitution_velocity);
      } else {
        lambda = vcp->m_normal_mass * rhs;
      }
    }
    // apply normal Impulse
    if(is_collision) {
      real new_impulse = b3_max(vc->m_normal_collision_impulse + lambda, real(0.0));
      lambda = new_impulse - vc->m_normal_collision_impulse;
      vc->m_normal_collision_impulse = new_impulse;
    } else {
      real new_impulse = b3_max(vc->m_normal_contact_impulse + lambda, real(0.0));
      lambda = new_impulse - vc->m_normal_contact_impulse;
      vc->m_normal_contact_impulse = new_impulse;
    }
    b3Vector3r impluse = lambda * vc->m_normal;

    v_a = v_a - vc->m_inv_mass_a * impluse;
    w_a = w_a - vc->m_inv_I_a * vc->m_ra.cross(impluse);
    v_b = v_b + vc->m_inv_mass_b * impluse;
    w_b = w_b + vc->m_inv_I_b * vc->m_rb.cross(impluse);
*/////////////////////////////////////////////////////////////////////////////////////////////////////////
        for (int32 j = 0; j < vc->m_point_count; ++j) {
            b3VelocityConstraintPoint* vcp = vc->m_points + j;

            b3Vector3r v_rel = v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra);
            real rhs = -v_rel.dot(vc->m_normal);

            real lambda = 0;
            // TODO:
            if(is_collision) {
                lambda = vcp->m_normal_mass * (rhs + vcp->m_rhs_restitution_velocity);
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
            b3Vector3r impulse = lambda * vc->m_normal;

            v_a = v_a - vc->m_inv_mass_a * impulse;
            w_a = w_a - vc->m_inv_I_a * vcp->m_ra.cross(impulse);
            v_b = v_b + vc->m_inv_mass_b * impulse;
            w_b = w_b + vc->m_inv_I_b * vcp->m_rb.cross(impulse);
        }
        m_vs[vc->m_index_a] = v_a;
        m_vs[vc->m_index_b] = v_b;
        if (is_collision) {
            m_ws[vc->m_index_a] = w_a;
            m_ws[vc->m_index_b] = w_b;
        }
    }
}


void b3VelocitySolver::correct_penetration()
{
    for(int32 i = 0; i < m_contact_count; ++i) {
        b3ContactVelocityConstraint *vc = m_velocity_constraints + i;

        // TODO: Check this way is useful. And angle?
        if (vc->m_penetration < 0) {
            b3Vector3r p_a = m_ps[vc->m_index_a];
            b3Vector3r p_b = m_ps[vc->m_index_b];

            b3Vector3r position_correction = vc->m_normal * vc->m_penetration;

            // at first, two bodies are not static.
            // if two bodies are dynamic, all need to fix penetration.
            // if one body is static, the dynamic body need to fix penetration * 2.
            // because when generate manifold, the penetration is equally distributed to two bodies.
            if(m_bodies[vc->m_index_a]->get_type() == b3BodyType::b3_dynamic_body &&
               m_bodies[vc->m_index_b]->get_type() == b3BodyType::b3_dynamic_body) {
                p_a += vc->m_normal * vc->m_penetration;
                p_b -= vc->m_normal * vc->m_penetration;
            } else if(m_bodies[vc->m_index_a]->get_type() == b3BodyType::b3_dynamic_body) {
                p_a += position_correction * real(2.0);
            } else if(m_bodies[vc->m_index_b]->get_type() == b3BodyType::b3_dynamic_body) {
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

