
#include "solver/b3_si_solver.hpp"

#include "dynamics/b3_island.hpp"
#include "dynamics/b3_body.hpp"

#include "collision/b3_fixture.hpp"
#include "collision/b3_collision.hpp"
#include "collision/b3_contact.hpp"

#include "common/b3_time_step.hpp"

#include <spdlog/spdlog.h>

//b3SISolver::b3SISolver(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step):
//  b3Solver(block_allocator, island, step)
//{
//  for(int32 i = 0; i < m_contact_count; ++i) {
//    b3Contact* contact = m_contacts[i];
//
//    b3Fixture *fixture_a = contact->get_fixture_a();
//    b3Fixture *fixture_b = contact->get_fixture_b();
//
//    b3Body *body_a = fixture_a->get_body();
//    b3Body *body_b = fixture_b->get_body();
//    b3Manifold *manifold = contact->get_manifold();
//
//    int32 point_count = manifold->m_point_count;
//
//    b3_assert(point_count > 0);
//
//    b3ContactVelocityConstraint* vc = m_velocity_constraints + i;
//    vc->m_restitution = contact->get_restitution();
//    vc->m_friction = contact->get_friction();
//    vc->m_contact_index = i;
//    vc->m_point_count = point_count;
//
//    vc->m_normal = manifold->m_local_normal;
//
//    vc->m_index_a = body_a->get_island_index();
//    vc->m_mass_a = body_a->get_mass();
//    vc->m_inv_mass_a = body_a->get_inv_mass();
//    vc->m_I_a = body_a->get_inertia();
//    vc->m_inv_I_a = body_a->get_inv_inertia();
//
//    vc->m_index_b = body_b->get_island_index();
//    vc->m_mass_b = body_b->get_mass();
//    vc->m_inv_mass_b = body_b->get_inv_mass();
//    vc->m_I_b = body_b->get_inertia();
//    vc->m_inv_I_b = body_b->get_inv_inertia();
//
//    vc->m_penetration = manifold->m_penetration;
//
//    vc->m_ra = b3Vector3r::zero();
//    vc->m_rb = b3Vector3r::zero();
//    // the center of body in the world frame
//    b3Vector3r center_a = body_a->get_pose().transform(body_a->get_local_center());
//    b3Vector3r center_b = body_b->get_pose().transform(body_b->get_local_center());
//
//    for (int32 j = 0; j < point_count; j++) {
//      b3VelocityConstraintPoint *vcp = vc->m_points + j;
//      b3ManifoldPoint *manifold_point = manifold->m_points + j;
//
//      vcp->m_ra = manifold_point->m_local_point - center_a;
//      vcp->m_rb = manifold_point->m_local_point - center_b;
//      // vcp->m_rhs_penetration = manifold->m_penetration;
//      // TODO: warm start
//    }
//  }
//}


void b3SISolver::init_velocity_constraints()
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
            double jmj = vc->m_inv_mass_a + vc->m_inv_mass_b +
            (ra_n * vc->m_inv_I_a).dot(ra_n) +
            (rb_n * vc->m_inv_I_b).dot(rb_n);

	        vcp->m_normal_mass = jmj > 0 ? 1.0 / jmj : 0;

            // 1. Mv+ = Mv + J^T * lambda ==> Jv+ = Jv + JM_invJ^T * lambda
            // 2. Jv+ = J(-ev)
            // ===> JM_invJ^T * lambda = -eJv - Jv
            double v_rel = vc->m_normal.dot(v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra));
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


int b3SISolver::solve()
{
    init_velocity_constraints();

    // collision
    for(int32 i = 0; i < m_timestep->m_velocity_iterations; ++i) {
        solve_velocity_constraints(true);
    }
    // fix penetration
    correct_penetration();

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
    solve_friction_constraints();

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


void b3SISolver::solve_velocity_constraints(bool is_collision)
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


void b3SISolver::correct_penetration()
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


void b3SISolver::solve_friction_constraints()
{
    for (int i = 0; i < m_contact_count; ++i) {
        b3ContactVelocityConstraint *vc = m_velocity_constraints + i;

        b3Vector3r v_a = m_vs[vc->m_index_a];
        b3Vector3r w_a = m_ws[vc->m_index_a];
        b3Vector3r v_b = m_vs[vc->m_index_b];
        b3Vector3r w_b = m_ws[vc->m_index_b];

        bool all_zero = true;
        for (int j = 0; j < vc->m_point_count; ++j) {
            if (vc->m_points[j].m_normal_contact_impulse != 0) {
                all_zero = false;
                break;
            }
        }
        real not_apply_support_impulse = 0;
        if (all_zero) {
            b3Body* b = m_bodies[vc->m_index_b];
            if(vc->m_inv_mass_b == 0) {
                b = m_bodies[vc->m_index_a];
            }
            not_apply_support_impulse = (b->get_gravity() + b->get_force()).dot(vc->m_normal);
            not_apply_support_impulse = b3_abs(not_apply_support_impulse * m_timestep->m_dt) / vc->m_point_count;
        }

        b3Vector3r points_ra[8];
        b3Vector3r points_rb[8];
        for (int k = 0; k < vc->m_point_count; ++k) {
            points_ra[k] = vc->m_points[k].m_ra;
            points_rb[k] = vc->m_points[k].m_rb;
        }

        for (int i = 0; i < vc->m_point_count; ++i) {
            b3VelocityConstraintPoint* vcp = vc->m_points + i;
            real support_impulse = vcp->m_normal_collision_impulse + vcp->m_normal_contact_impulse;
            real point_not_apply_support_impulse = all_zero ? not_apply_support_impulse : vcp->m_normal_contact_impulse;

            real max_friction_impulse = vc->m_friction * support_impulse;

            b3Vector3r v_rel = v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra);
            b3Vector3r v_rel_t = v_rel - vc->m_normal * v_rel.dot(vc->m_normal);

            // static friction, don't change the velocity.
            if (v_rel_t.is_zero()) {
                continue;
            }

            real v_rel_t_size = v_rel_t.length();
            b3Vector3r v_rel_t_dir = v_rel_t / v_rel_t_size;

            bool is_static = false;
            if (vc->m_inv_mass_b != 0) {
                real v_real_zero_impulse = v_rel_t_size * vc->m_mass_b;
                if(v_real_zero_impulse < max_friction_impulse) {
                    is_static = true;
                    max_friction_impulse = v_real_zero_impulse;
                }
                solve_friction_help(vc->m_index_a, vc->m_index_b,
                                    vc->m_normal, v_rel_t_dir, is_static,
                                    max_friction_impulse, not_apply_support_impulse, vc->m_ra, vc->m_rb,
                                    points_ra, points_rb, vc->m_point_count, vc);
            } else {
                real v_real_zero_impulse = v_rel_t_size * vc->m_mass_a;
                if(v_real_zero_impulse < max_friction_impulse) {
                    is_static = true;
                    max_friction_impulse = v_real_zero_impulse;
                }
                solve_friction_help(vc->m_index_b, vc->m_index_a,
                                    -vc->m_normal, -v_rel_t_dir, is_static,
                                    max_friction_impulse, not_apply_support_impulse, vc->m_rb, vc->m_ra,
                                    points_rb, points_ra, vc->m_point_count, vc);
            }
        }

        // Can the friction impulse make the body rest ?
        // At least one of body A and B is dynamic.
        // if b is static, it means any force is not work for it. we need analyze body a in body b view.
        // so the velocity direction need reverse.
        // if the change velocity is greater than the relative velocity, prove the friction is static.

    }
}


// from body a view to analyze body b.
// the normal is body a to body b.
// the tangent is the relative tangent velocity direction.
// the friction impulse direction is -tangent.
void b3SISolver::solve_friction_help(
    int index_a, int index_b, const b3Vector3r& normal, const b3Vector3r& tangent, bool is_static,
    real friction_impulse, real support_impulse, const b3Vector3r& ra, const b3Vector3r& rb,
    b3Vector3r* points_ra, b3Vector3r* points_rb, int point_count, b3ContactVelocityConstraint* vc)
{

    b3Vector3r v_a = m_vs[index_a];
    b3Vector3r v_b = m_vs[index_b];
    v_a += vc->m_inv_mass_a * (friction_impulse * tangent);
    v_b -= vc->m_inv_mass_b * (friction_impulse * tangent);
    m_vs[index_a] = v_a;
    m_vs[index_b] = v_b;
  // if the body friciton is static, we need tangent velocity is zero.
  // and the body should not rotate. so the angular velocity is don't change.
//  if (is_static) {
//    return;
//  }

  // First consider body b 2D cut plane.
  // the x-axis is the tangent velocity direction.
  // the y-axis is the contact normal direction.
  // and the origin point is the center of body b(project to this plane).
    real h = b3_abs(rb.dot(normal));
    real max_b = rb.dot(tangent);
    real min_b = 0;
    if (max_b < min_b) {
        real temp = max_b;
        max_b = min_b;
        min_b = temp;
    }
    // If the torque is supported, the frictional moment can be corrected
    // Tf the support torque can correct the frictional torque.
    // the angular velocity not change.
    // TODO: if we need consider the support toque to minus the angular velocity.
    b3Vector3r w_b = m_ws[index_b];

    b3Vector3r torque_direction = normal.cross(tangent);
    // torque = I * w; and then get the torque direction component.
    // The direction is pointing inside the plane. maybe less than zero.
    real w_b_to_zero_torque_impulse = (vc->m_I_b * w_b).dot(torque_direction);
    // the friction torque direction is pointing inside the plane, so less than zero.
    w_b_to_zero_torque_impulse += friction_impulse * h;

    // the support force also produce torque = r cross support_force
    // r is [min_b, max_b] so the support torque is [min_b * support_impulse, max_b * support_impulse]
    // if w_b_to_zero_torque_impulse is in the range,  the angular velocity will be zero.
    real min_support_torque = min_b * support_impulse;
    real max_support_torque = max_b * support_impulse;
    real support_torque = w_b_to_zero_torque_impulse;
    // The angular velocity cannot be set to 0 by support torque, so the maximum torque is taken
    if (w_b_to_zero_torque_impulse > max_support_torque) {
        support_torque = max_support_torque;
    }
    if (w_b_to_zero_torque_impulse < min_support_torque) {
        support_torque = min_support_torque;
    }

    w_b += vc->m_inv_I_b * ((friction_impulse * h - support_torque) * torque_direction);
    m_ws[index_b] = w_b;

    // friction impulse for body a.
    if (vc->m_inv_mass_a == 0) {
        return;
    }

    // we need recompute h and b for body a.
    h = b3_abs(ra.dot(normal));
    max_b = ra.dot(tangent);
    min_b = 0;
    if (max_b < min_b) {
        real temp = max_b;
        max_b = min_b;
        min_b = temp;
    }

    b3Vector3r w_a = m_ws[index_a];
    // The direction is pointing inside the plane. maybe less than zero.
    real w_a_to_zero_torque_impulse = (vc->m_I_a * w_a).dot(torque_direction);
    // the friction works in the opposite direction
    w_a_to_zero_torque_impulse += friction_impulse * h;

    min_support_torque = max_b * support_impulse;
    max_support_torque = min_b * support_impulse;
    support_torque = w_a_to_zero_torque_impulse;
    if (w_a_to_zero_torque_impulse > max_support_torque) {
        support_torque = max_support_torque;
    }
    if (w_a_to_zero_torque_impulse < min_support_torque) {
        support_torque = min_support_torque;
    }

    w_a += vc->m_inv_I_a * ((friction_impulse * h - support_torque) * torque_direction);
    m_ws[index_a] = w_a;

}


void b3SISolver::init(b3BlockAllocator *block_allocator, b3Island *island, b3TimeStep *step) {

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
        vc->m_I_a = body_a->get_inertia();
        vc->m_inv_I_a = body_a->get_inv_inertia();

        vc->m_index_b = body_b->get_island_index();
        vc->m_mass_b = body_b->get_mass();
        vc->m_inv_mass_b = body_b->get_inv_mass();
        vc->m_I_b = body_b->get_inertia();
        vc->m_inv_I_b = body_b->get_inv_inertia();

        vc->m_penetration = manifold->m_penetration;

        vc->m_ra = b3Vector3r::zero();
        vc->m_rb = b3Vector3r::zero();
        // the center of body in the world frame
        b3Transformr T_a(body_a->get_position(), body_a->get_quaternion());
        b3Transformr T_b(body_b->get_position(), body_b->get_quaternion());
        b3Vector3r center_a = T_a.transform(body_a->get_local_center());
        b3Vector3r center_b = T_b.transform(body_b->get_local_center());

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