
#include "solver/b3_solver.hpp"

#include "dynamics/b3_island.hpp"
#include "dynamics/b3_body.hpp"
#include "collision/b3_collision.hpp"
#include "collision/b3_contact.hpp"
#include "common/b3_time_step.hpp"
#include "common/b3_block_allocator.hpp"
#include "collision/b3_fixture.hpp"

#include "spdlog/spdlog.h"
//////////////////////////////////////////
// In the tangent plane, t1 and t2 are counterclockwise
/*
 *   ^ t2
 *   |
 *   |
 *   n----> t1
 *
 */
static void b3_get_two_tangent_bases(const b3Vec3r& normal, b3Vec3r& t1, b3Vec3r& t2)
{
    if(b3_abs(normal.x) < b3_real_min && b3_abs(normal.y) < b3_real_min) {
        t1 = b3Vec3r(1, 0, 0);
        t2 = b3Vec3r(0 , 1, 0);
        return;
    }
    t1 = b3Vec3r(-normal.y, normal.x, 0);
    t2 = normal.cross(t1).normalized();
    t1 = t2.cross(normal).normalized();
}

/////////////////////////////////////////


void b3Solver::init(b3BlockAllocator *block_allocator, b3Island *island, b3TimeStep *step)
{
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
        vc->m_contact_index = i;
        vc->m_point_count = point_count;

        vc->m_normal = manifold->m_local_normal;
        b3_get_two_tangent_bases(vc->m_normal, vc->m_tangent1, vc->m_tangent2);

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

        vc->m_is_sphere[0] = (fixture_a->get_shape_type() == b3ShapeType::e_sphere);
        vc->m_is_sphere[1] = (fixture_b->get_shape_type() == b3ShapeType::e_sphere);
        //TODO: make 0.1 become a member of the sphere class.
        vc->m_radius[0] = (real)0.2 * fixture_a->get_shape()->get_radius();
        vc->m_inv_radius[0] = (real)1.0 / vc->m_radius[0];
        vc->m_radius[1] = (real)0.2 * fixture_b->get_shape()->get_radius();
        vc->m_inv_radius[1] = (real)1.0 / vc->m_radius[1];

        vc->m_inv_I_ab = (vc->m_inv_I_a + vc->m_inv_I_b).inverse();

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
    }

}


void b3Solver::write_states_back()
{
    for(int32 i = 0; i < m_body_count; ++i) {
        m_bodies[i]->set_position(m_ps[i]);
        m_bodies[i]->set_quaternion(m_qs[i]);
        m_bodies[i]->set_linear_velocity(m_vs[i]);
        m_bodies[i]->set_angular_velocity(m_ws[i]);
        // TODO: if we need SynchronizeTransform() ?
    }

    for(int32 i = 0; i < m_body_count; i++) {
        if(m_bodies[i]->get_type() == b3BodyType::b3_dynamic_body) {
            spdlog::log(spdlog::level::info, "count: {}, velocity: {}, {}, {}, {}, {}, {}",
                        m_contact_count,
                        m_bodies[i]->get_linear_velocity().x,
                        m_bodies[i]->get_linear_velocity().y,
                        m_bodies[i]->get_linear_velocity().z,
                        m_bodies[i]->get_angular_velocity().x,
                        m_bodies[i]->get_angular_velocity().y,
                        m_bodies[i]->get_angular_velocity().z
            );
        }
    }
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
        m_qs[i] = m_qs[i] + real(0.5) * m_timestep->m_dt * b3Quaternionr(0, m_ws[i]) * m_qs[i];
        m_qs[i].normalize();
    }

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

            real max_friction;
            if(is_collision) {
                max_friction = vc->m_friction * vcp->m_normal_collision_impulse;
            } else {
                max_friction = vc->m_friction * vcp->m_normal_contact_impulse;
            }

            float new_tangent1_lambda = b3_clamp(vcp->m_tangent1_impulse + tangent1_lambda, -max_friction, max_friction);
            tangent1_lambda = new_tangent1_lambda - vcp->m_tangent1_impulse;
            vcp->m_tangent1_impulse = new_tangent1_lambda;

            float tangent2_lambda = vcp->m_tangent2_mass * (-vt2);

            float new_tangent2_lambda = b3_clamp(vcp->m_tangent2_impulse + tangent2_lambda, -max_friction, max_friction);
            tangent2_lambda = new_tangent2_lambda - vcp->m_tangent2_impulse;
            vcp->m_tangent2_impulse = new_tangent2_lambda;

            b3Vec3r impulse = tangent1_lambda * vc->m_tangent1 + tangent2_lambda * vc->m_tangent2;

            v_a = v_a - vc->m_inv_mass_a * impulse;
            w_a = w_a - vc->m_inv_I_a * vcp->m_ra.cross(impulse);
            v_b = v_b + vc->m_inv_mass_b * impulse;
            w_b = w_b + vc->m_inv_I_b * vcp->m_rb.cross(impulse);
        }

        for (int32 j = 0; j < vc->m_point_count; ++j) {
            b3VelocityConstraintPoint *vcp = vc->m_points + j;

            // TODO: if we enable this ?
            // TODO: Borrowed from《Nonconvex Rigid Bodies with Stacking》
            real w_rel = (w_b - w_a).dot(vc->m_normal);
            b3Vec3r lambda_torque = vc->m_inv_I_ab * vc->m_normal * (vc->m_target_normal_angular_velocity - w_rel);
            w_b = w_b + vc->m_inv_I_b * lambda_torque;
            w_a = w_a - vc->m_inv_I_a * lambda_torque;

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

            v_a = v_a - vc->m_inv_mass_a * impulse;
            v_b = v_b + vc->m_inv_mass_b * impulse;
            w_a = w_a - vc->m_inv_I_a * vcp->m_ra.cross(impulse);
            w_b = w_b + vc->m_inv_I_b * vcp->m_rb.cross(impulse);
        }

        m_vs[vc->m_index_a] = v_a;
        m_vs[vc->m_index_b] = v_b;
        m_ws[vc->m_index_a] = w_a;
        m_ws[vc->m_index_b] = w_b;

        // solve_sphere_angular_velocity(vc);
    }
}


void b3Solver::solve_sphere_angular_velocity(b3ContactVelocityConstraint *vc)
{
    // Try correcting the angular velocity along the t1 and t2 axes
    if (vc->m_is_sphere[0]) {
        b3_assert(vc->m_point_count == 1);
        // for sphere a
        b3Vec3r w = m_ws[vc->m_index_a];

        // Iw * 1/d = N
        b3Vec3r Iw_to_zero = vc->m_I_a * w * m_timestep->m_inv_dt * vc->m_inv_radius[0];

        real Iw_t[2];
        Iw_t[0] = Iw_to_zero.dot(vc->m_tangent1);
        Iw_t[1] = Iw_to_zero.dot(vc->m_tangent2);
        real Iw_n = Iw_to_zero.dot(vc->m_normal);

        // boundary is [-boundary, boundary]
        real boundary = vc->m_points->m_normal_collision_impulse + vc->m_points->m_normal_contact_impulse;
        // for t1 and t2, now use some, so boundary is [-boundary - use, boundary - use]
        for(int i = 0; i < 2; i++) {
            if(Iw_t[i] > boundary + vc->m_use_support_lambda[0][i]) {
                Iw_t[i] = Iw_t[i] - boundary - vc->m_use_support_lambda[0][i];
                vc->m_use_support_lambda[0][i] = -boundary;
            } else if(Iw_t[i] < vc->m_use_support_lambda[0][i] - boundary) {
                Iw_t[i] = Iw_t[i] + boundary - vc->m_use_support_lambda[0][i];
                vc->m_use_support_lambda[0][i] = boundary;
            } else {
                vc->m_use_support_lambda[0][i] -= Iw_t[i];
                Iw_t[i] = 0;
            }
        }

        w = vc->m_inv_I_a * (Iw_n * vc->m_normal + Iw_t[0] * vc->m_tangent1 + Iw_t[1] * vc->m_tangent2);
        w = vc->m_radius[0] * w;
        m_ws[vc->m_index_a] = w;
    }

    if (vc->m_is_sphere[1]) {
        b3_assert(vc->m_point_count == 1);
        // for sphere b
        b3Vec3r w = m_ws[vc->m_index_b];
        // Iw * 1/d = N
        b3Vec3r Iw_to_zero = vc->m_I_b * w * vc->m_inv_radius[1];

        real Iw_t[2];
        Iw_t[0] = Iw_to_zero.dot(vc->m_tangent1);
        Iw_t[1] = Iw_to_zero.dot(vc->m_tangent2);
        real Iw_n = Iw_to_zero.dot(vc->m_normal);

        // boundary is [-boundary, boundary]
        real boundary = vc->m_points->m_normal_collision_impulse + vc->m_points->m_normal_contact_impulse;
        // for t1 and t2, now use some, so boundary is [-boundary - use, boundary - use]
        for(int i = 0; i < 2; i++) {
            if(Iw_t[i] > boundary + vc->m_use_support_lambda[1][i]) {
                Iw_t[i] = Iw_t[i] - boundary - vc->m_use_support_lambda[1][i];
                vc->m_use_support_lambda[1][i] = -boundary;
            } else if(Iw_t[i] < vc->m_use_support_lambda[1][i] - boundary) {
                Iw_t[i] = Iw_t[i] + boundary - vc->m_use_support_lambda[1][i];
                vc->m_use_support_lambda[1][i] = boundary;
            } else {
                vc->m_use_support_lambda[1][i] -= Iw_t[i];
                Iw_t[i] = 0;
            }
        }

        w = vc->m_inv_I_b * (Iw_n * vc->m_normal + Iw_t[0] * vc->m_tangent1 + Iw_t[1] * vc->m_tangent2);
        w = vc->m_radius[1] * w;
        m_ws[vc->m_index_b] = w;
    }
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

        b3Vec3r w_rel = w_b - w_a;
        vc->m_target_normal_angular_velocity = w_rel.dot(vc->m_normal);
        vc->m_target_normal_angular_velocity -= m_timestep->m_dt * (1 - vc->m_restitution) * vc->m_target_normal_angular_velocity;

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

            vcp->m_normal_mass = effective_mass > 0 ? static_cast<real>(1) / effective_mass : 0;

            b3Vec3r ra_t1 = vcp->m_ra.cross(vc->m_tangent1);
            b3Vec3r rb_t1 = vcp->m_rb.cross(vc->m_tangent1);
            effective_mass = vc->m_inv_mass_a + vc->m_inv_mass_b + (ra_t1 * vc->m_inv_I_a).dot(ra_t1)
                             + (rb_t1 * vc->m_inv_I_b).dot(rb_t1);

            vcp->m_tangent1_mass = effective_mass > 0 ? static_cast<real>(1) / effective_mass : 0;

            b3Vec3r ra_t2 = vcp->m_ra.cross(vc->m_tangent2);
            b3Vec3r rb_t2 = vcp->m_rb.cross(vc->m_tangent2);
            effective_mass = vc->m_inv_mass_a + vc->m_inv_mass_b + (ra_t2 * vc->m_inv_I_a).dot(ra_t2)
                             + (rb_t2 * vc->m_inv_I_b).dot(rb_t2);
            vcp->m_tangent2_mass = effective_mass > 0 ? static_cast<real>(1) / effective_mass : 0;

            real y = (vc->m_inv_I_b * vcp->m_rb.cross(vc->m_tangent2)).cross(vcp->m_rb).dot(vc->m_tangent2);
            real y_copy = (rb_t2 * vc->m_inv_I_b).dot(rb_t2);
            // 1. Mv+ = Mv + J^T * lambda ==> Jv+ = Jv + JM_invJ^T * lambda
            // 2. Jv+ = J(-ev)
            // ===> JM_invJ^T * lambda = -eJv - Jv
            real v_rel = vc->m_normal.dot(v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra));
            // m_rhs_restitution_velocity is eJv
            vcp->m_rhs_restitution_velocity = -vc->m_restitution * v_rel;

            vcp->m_normal_collision_impulse = 0.0;
            vcp->m_normal_contact_impulse = 0.0;
            vcp->m_tangent1_impulse = 0.0;
            vcp->m_tangent2_impulse = 0.0;
        }
        vc->m_normal_contact_impulse = 0.0;
        vc->m_normal_collision_impulse = 0.0;
        vc->m_use_support_lambda[0][0] = 0.0;
        vc->m_use_support_lambda[0][1] = 0.0;
        vc->m_use_support_lambda[1][0] = 0.0;
        vc->m_use_support_lambda[1][1] = 0.0;

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