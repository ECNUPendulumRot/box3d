
#include "solver/b3_si_solver.hpp"

#include "dynamics/b3_island.hpp"
#include "dynamics/b3_body.hpp"

#include "collision/b3_fixture.hpp"
#include "collision/b3_collision.hpp"
#include "collision/b3_contact.hpp"

#include "common/b3_time_step.hpp"

#include <spdlog/spdlog.h>

b3SISolver::b3SISolver(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step):
  b3Solver(block_allocator, island, step)
{
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
    b3Vector3r center_a = body_a->get_pose().transform(body_a->get_local_center());
    b3Vector3r center_b = body_b->get_pose().transform(body_b->get_local_center());

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


void b3SISolver::init_velocity_constraints()
{
  for(int32 i = 0; i < m_contact_count; ++i) {
    b3ContactVelocityConstraint* vc = m_velocity_constraints + i;
    int32 index_a = vc->m_index_a;
    int32 index_b = vc->m_index_b;

    b3Vector3r v_a = m_velocities[index_a].linear();
    b3Vector3r w_a = m_velocities[index_a].angular();

    b3Vector3r v_b = m_velocities[index_b].linear();
    b3Vector3r w_b = m_velocities[index_b].angular();

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


int b3SISolver::solve(int type)
{
  init_velocity_constraints();

  for(int i = 0; i < m_body_count; ++i) {
    if(m_bodies[i]->get_type() != b3BodyType::b3_static_body) {
      spdlog::log(spdlog::level::info, "start position: {}, {}, {}, velocity: {}, {}, {}, angular velocity: {}, {}, {}",
                   m_positions[i].linear().x(), m_positions[i].linear().y(), m_positions[i].linear().z(),
                   m_velocities[i].linear().x(), m_velocities[i].linear().y(), m_velocities[i].linear().z(),
                   m_velocities[i].angular().x(), m_velocities[i].angular().y(), m_velocities[i].angular().z());
    }
  }

  // collision
  for(int32 i = 0; i < m_timestep->m_velocity_iterations; ++i) {
    solve_velocity_constraints(true);
  }
  // fix penetration
  correct_penetration();

  // velocity update
  for(int32 i = 0; i < m_body_count; ++i) {
    b3Body *b = m_bodies[i];
    b3Vector3r v = m_velocities[i].linear();
    b3Vector3r w = m_velocities[i].angular();
    //store the velocity before apply force
    m_velocities_w_f[i].set_linear(v);
    m_velocities_w_f[i].set_angular(w);

    v += m_timestep->m_dt * b->get_inv_mass() * (b->get_force() + b->get_gravity());
    w += m_timestep->m_dt * b->get_inv_inertia() * b->get_torque();

    m_velocities[i].set_linear(v);
    m_velocities[i].set_angular(w);
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

  if (type == 0) {
    for (int32 i = 0; i < m_body_count; ++i) {
      m_positions[i].set_linear(m_positions[i].linear() + m_velocities[i].linear() * m_timestep->m_dt);
      m_positions[i].set_angular(m_positions[i].angular() + m_velocities[i].angular() * m_timestep->m_dt);
    }
  } else if (type == 1) {
    for (int32 i = 0; i < m_body_count; ++i) {
      m_positions[i].set_linear(m_positions[i].linear() +
                                (m_velocities[i].linear() + m_velocities_w_f[i].linear()) *
                                m_timestep->m_dt * real(0.5));
      m_positions[i].set_angular(m_positions[i].angular() +
                                 (m_velocities[i].angular() + m_velocities_w_f[i].angular()) *
                                 m_timestep->m_dt * real(0.5));
    }
  }
	// copy state buffers back to the bodies.
	write_states_back();

	return 0;
}


void b3SISolver::solve_velocity_constraints(bool is_collision)
{
  for (int32 i = 0; i < m_contact_count; ++i) {
    b3ContactVelocityConstraint *vc = m_velocity_constraints + i;

    b3Vector3r v_a = m_velocities[vc->m_index_a].linear();
    b3Vector3r w_a = m_velocities[vc->m_index_a].angular();

    b3Vector3r v_b = m_velocities[vc->m_index_b].linear();
    b3Vector3r w_b = m_velocities[vc->m_index_b].angular();

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
    m_velocities[vc->m_index_a].set_linear(v_a);
    m_velocities[vc->m_index_b].set_linear(v_b);
    if (is_collision) {
      m_velocities[vc->m_index_a].set_angular(w_a);
      m_velocities[vc->m_index_b].set_angular(w_b);
    }
  }
}


void b3SISolver::correct_penetration()
{
  for(int32 i = 0; i < m_contact_count; ++i) {
    b3ContactVelocityConstraint *vc = m_velocity_constraints + i;

    // TODO: Check this way is useful. And angle?
    if (vc->m_penetration < 0) {
      b3Vector3r p_a = m_positions[vc->m_index_a].linear();
      b3Vector3r p_b = m_positions[vc->m_index_b].linear();

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

      m_positions[vc->m_index_a].set_linear(p_a);
      m_positions[vc->m_index_b].set_linear(p_b);

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

    b3Vector3r v_a = m_velocities[vc->m_index_a].linear();
    b3Vector3r w_a = m_velocities[vc->m_index_a].angular();
    b3Vector3r v_b = m_velocities[vc->m_index_b].linear();
    b3Vector3r w_b = m_velocities[vc->m_index_b].angular();

    // compute the support force for b from a
    real support_impulse = 0;
    real not_apply_support_impulse = 0;
    for (int j = 0; j < vc->m_point_count; ++j) {
      not_apply_support_impulse += vc->m_points[j].m_normal_contact_impulse;
      support_impulse += vc->m_points[j].m_normal_collision_impulse;
    }

    support_impulse += not_apply_support_impulse;
    if (not_apply_support_impulse == 0) {
      b3Body* b = m_bodies[vc->m_index_b];
      if(vc->m_inv_mass_b == 0) {
        b = m_bodies[vc->m_index_a];
      }
      not_apply_support_impulse = (b->get_gravity() + b->get_force()).dot(vc->m_normal);
      not_apply_support_impulse = b3_abs(not_apply_support_impulse * m_timestep->m_dt);
    }

    // The support impulse is exerted by body A to body B.
    // support_impulse = support_impulse / vc->m_point_count;
    real max_friction_impulse = vc->m_friction * support_impulse;

    // if the relative velocity is non-zero, must exist friction force.
    // Previous, we don't consider friction, and now we consider it. so the velocity now is must >= real case.
    // TODO: now only consider one contact point.
    // b3Vector3r v_rel = v_b + w_b.cross(vc->m_rb) - v_a - w_a.cross(vc->m_ra);
    b3Vector3r v_rel = v_b - v_a;
    // tangent relative velocity, we get the direction and the size.
    b3Vector3r v_rel_t = v_rel - vc->m_normal * v_rel.dot(vc->m_normal);

    // static friction, don't change the velocity.
    if (v_rel_t.is_zero()) {
      continue;
    }

    real v_rel_t_size = v_rel_t.length();
    b3Vector3r v_rel_t_dir = v_rel_t / v_rel_t_size;

    // Can the friction impulse make the body rest ?
    // At least one of body A and B is dynamic.
    // if b is static, it means any force is not work for it. we need analyze body a in body b view.
    // so the velocity direction need reverse.
    // if the change velocity is greater than the relative velocity, prove the friction is static.
    b3Vector3r points_ra[8];
    b3Vector3r points_rb[8];
    for (int k = 0; k < vc->m_point_count; ++k) {
      points_ra[k] = vc->m_points[k].m_ra;
      points_rb[k] = vc->m_points[k].m_rb;
    }

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

  b3Vector3r v_a = m_velocities[index_a].linear();
  b3Vector3r v_b = m_velocities[index_b].linear();
  v_a += vc->m_inv_mass_a * (friction_impulse * tangent);
  v_b -= vc->m_inv_mass_b * (friction_impulse * tangent);
  m_velocities[index_a].set_linear(v_a);
  m_velocities[index_b].set_linear(v_b);
  // if the body friciton is static, we need tangent velocity is zero.
  // and the body should not rotate. so the angular velocity is don't change.
  if (is_static) {
    return;
  }

  // First consider body b 2D cut plane.
  // the x-axis is the tangent velocity direction.
  // the y-axis is the contact normal direction.
  // and the origin point is the center of body b(project to this plane).
  real h = b3_abs(rb.dot(normal));
  real max_b = points_rb[0].dot(tangent);
  real min_b = points_rb[0].dot(tangent);
  real b = points_rb[0].dot(tangent);
  for (int k = 1; k < point_count; ++k) {
    real temp = points_rb[k].dot(tangent);
    b = b3_max(temp, b);
    max_b = b3_max(temp, max_b);
    min_b = b3_min(temp, min_b);
  }
  // If the torque is supported, the frictional moment can be corrected
  // Tf the support torque can correct the frictional torque.
  // the angular velocity not change.
  // TODO: if we need consider the support toque to minus the angular velocity.
  b3Vector3r w_b = m_velocities[index_b].angular();

  b3Vector3r torque_direction = normal.cross(tangent);
  // torque = I * w; and then get the torque direction component.
  // The direction is pointing inside the plane. if it less than zero, the direction is reverse.
  real w_b_to_zero_torque_impulse = -(vc->m_I_b * w_b).dot(torque_direction);
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

  w_b += vc->m_inv_I_b * ((support_torque - friction_impulse * h) * tangent);
  m_velocities[index_b].set_angular(w_b);

//
//  if (vc->m_friction * h > b) {
//    // also need change linear velocity
//    b3Vector3r torque_impulse = rb.cross(-friction_impulse * tangent);
//    // support force also produce torque.
//    torque_impulse += (b * tangent).cross(support_impulse * normal);
//
//    b3Vector3r w_b = m_velocities[index_b].angular();
//    w_b += vc->m_inv_I_b * torque_impulse;
//    m_velocities[index_b].set_angular(w_b);
//  }

  // friction impulse for body a.
  if (vc->m_inv_mass_a == 0) {
    return;
  }


  // we need recompute h and b for body a.
  h = b3_abs(ra.dot(normal));
  b = points_ra[0].dot(tangent);
  for (int k = 1; k < point_count; ++k) {
    real temp = points_ra[k].dot(tangent);
    if (temp < b) {
      b = temp;
    }
  }
  b = b3_abs(b);

  if(vc->m_friction * h > b) {
    b3Vector3r torque_impulse = ra.cross(friction_impulse * tangent);
    torque_impulse += (-b * tangent).cross(-support_impulse * normal);

    b3Vector3r w_a = m_velocities[index_a].angular();
    w_a += vc->m_inv_I_a * torque_impulse;
    m_velocities[index_a].set_angular(w_a);
  }

}


///////////////////////////////////////////////////////////////////////////////////////////////
/*
void b3SISolver::solve_friction_constraints()
{
  for (int32 i = 0; i < m_contact_count; ++i) {
    b3ContactVelocityConstraint* vc = m_velocity_constraints + i;

    b3Vector3r v_a = m_velocities[vc->m_index_a].linear();
    b3Vector3r w_a = m_velocities[vc->m_index_a].angular();

    b3Vector3r v_b = m_velocities[vc->m_index_b].linear();
    b3Vector3r w_b = m_velocities[vc->m_index_b].angular();

    for (int i = 0; i < vc->m_point_count; ++i) {
      b3VelocityConstraintPoint* vcp = vc->m_points + i;

      // vc->m_friction is the friction coefficient.
      // vc->m_normal_contact_impulse is the normal impulse = ft.
      real max_friction = vc->m_friction * vcp->m_normal_contact_impulse;

      // In the view of body a, body a is static, the velocity of body b is v_rel
      b3Vector3r v_rel = v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra);
      // In the tangent plane, the velocity is v_rel_t
      b3Vector3r v_rel_t = v_rel - vc->m_normal * v_rel.dot(vc->m_normal);

      // if the tangent velocity is zero ===> static friction
      if(v_rel_t.is_zero()) {
        continue;
      }
      // get the tangent direction and size
      real v_rel_t_ = v_rel_t.length();
      b3Vector3r tangent = v_rel_t / v_rel_t_;

      b3Vector3r friction_v;

      // a and b at least have one dynamic body.
      // if b is static, we need go to body b view, so the tangent velocity direction is -tangent.
      // and then we get the max_friction.( (friction coefficient) * ft / m )
      if(vc->m_inv_mass_b != 0) {
        max_friction = max_friction * vc->m_inv_mass_b;
      } else {
        max_friction = max_friction * vc->m_inv_mass_a;
        tangent = -tangent;
      }
      // we don't consider the friction when we update the velocity of bodies.
      // So now we consider the friction to try to correct this error.
      // if max_friction is greater than v_rel_t_, mean that the velocity will be zero by friction.
      // else we use the max_friction to correct the velocity.
      if(max_friction >= v_rel_t_) {
        friction_v = v_rel_t_ * tangent;
      } else {
        friction_v = max_friction * tangent;
      }

      if(vc->m_inv_mass_b != 0) {
        friction_v /= vc->m_inv_mass_b;
        if(vc->m_inv_mass_a != 0) {
          v_a = v_a + vc->m_inv_mass_a * friction_v;
          w_a = w_a + vc->m_inv_I_a * vcp->m_ra.cross(friction_v);
        }
        v_b = v_b - vc->m_inv_mass_b * friction_v;
        w_b = w_b - vc->m_inv_I_b * vcp->m_rb.cross(friction_v);
      } else {
        friction_v /= vc->m_inv_mass_a;
        v_a = v_a - vc->m_inv_mass_a * friction_v;
        w_a = w_a - vc->m_inv_I_a * vcp->m_ra.cross(friction_v);
      }
    }


//////////////////////////////////////////////////////////////////////////
/*
    // vc->m_friction is the friction coefficient.
    // vc->m_normal_contact_impulse is the normal impulse = ft.
    real max_friction = vc->m_friction * vc->m_normal_contact_impulse;

    // In the view of body a, body a is static, the velocity of body b is v_rel
    b3Vector3r v_rel = v_b + w_b.cross(vc->m_rb) - v_a - w_a.cross(vc->m_ra);
    // In the tangent plane, the velocity is v_rel_t
    b3Vector3r v_rel_t = v_rel - vc->m_normal * v_rel.dot(vc->m_normal);

    // if the tangent velocity is zero ===> static friction
    if(v_rel_t.is_zero()) {
      continue;
    }
    // get the tangent direction and size
    real v_rel_t_ = v_rel_t.length();
    b3Vector3r tangent = v_rel_t / v_rel_t_;

    b3Vector3r friction_v;

    // a and b at least have one dynamic body.
    // if b is static, we need go to body b view, so the tangent velocity direction is -tangent.
    // and then we get the max_friction.( (friction coefficient) * ft / m )
    if(vc->m_inv_mass_b != 0) {
      max_friction = max_friction * vc->m_inv_mass_b;
    } else {
      max_friction = max_friction * vc->m_inv_mass_a;
      tangent = -tangent;
    }
    // we don't consider the friction when we update the velocity of bodies.
    // So now we consider the friction to try to correct this error.
    // if max_friction is greater than v_rel_t_, mean that the velocity will be zero by friction.
    // else we use the max_friction to correct the velocity.
    if(max_friction >= v_rel_t_) {
      friction_v = v_rel_t_ * tangent;
    } else {
      friction_v = max_friction * tangent;
    }

    if(vc->m_inv_mass_b != 0) {
      friction_v /= vc->m_inv_mass_b;
      if(vc->m_inv_mass_a != 0) {
        v_a = v_a + vc->m_inv_mass_a * friction_v;
        w_a = w_a + vc->m_inv_I_a * vc->m_ra.cross(friction_v);
      }
      v_b = v_b - vc->m_inv_mass_b * friction_v;
      w_b = w_b - vc->m_inv_I_b * vc->m_rb.cross(friction_v);
    } else {
      friction_v /= vc->m_inv_mass_a;
      v_a = v_a - vc->m_inv_mass_a * friction_v;
      w_a = w_a - vc->m_inv_I_a * vc->m_ra.cross(friction_v);
    }
*/////////////////////////////////////////////////////////////////////////////////
//
//    m_velocities[vc->m_index_a].set_linear(v_a);
//    m_velocities[vc->m_index_a].set_angular(w_a);
//    m_velocities[vc->m_index_b].set_linear(v_b);
//    m_velocities[vc->m_index_b].set_angular(w_b);
//  }
//}
