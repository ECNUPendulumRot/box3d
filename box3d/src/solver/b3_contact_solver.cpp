
#include "solver/b3_contact_solver.hpp"
#include "collision/b3_fixture.hpp"
#include "collision/b3_contact.hpp"
#include "dynamics/b3_body.hpp"
#include "solver/b3_lemke.hpp"
#include "common/b3_time_step.hpp"
#include <spdlog/spdlog.h>

static bool g_block_solve = false;


b3ContactSolver::b3ContactSolver(b3ContactSolverDef *def)
{
    m_step = def->step;
    m_contacts = def->contacts;
    m_count = def->count;
    m_ps = def->ps;
    m_qs = def->qs;
    m_vs = def->vs;
    m_ws = def->ws;
    m_block_allocator = def->block_allocator;


    m_velocity_constraints = (b3ContactVelocityConstraint*)m_block_allocator->allocate(m_count * sizeof(b3ContactVelocityConstraint));
    m_position_constraints = (b3ContactPositionConstraint*)m_block_allocator->allocate(m_count * sizeof(b3ContactPositionConstraint));

    for(int32 i = 0; i < m_count; ++i) {

        b3Contact* contact = m_contacts[i];

        b3Fixture* fixture_a = contact->get_fixture_a();
        b3Fixture* fixture_b = contact->get_fixture_b();

        b3Body* body_a = fixture_a->get_body();
        b3Body* body_b = fixture_b->get_body();

        b3Manifold* manifold = contact->get_manifold();

        int32 point_count = manifold->m_point_count;

        b3_assert(point_count > 0);
        //////////////////////////// Velocity Constraints ////////////////////////////
        b3ContactVelocityConstraint* vc = m_velocity_constraints + i;
        vc->m_restitution = contact->get_restitution();
        vc->m_friction = contact->get_friction();
        vc->m_contact_index = i;
        vc->m_point_count = point_count;
        vc->m_restitution_threshold = contact->get_restitution_threshold();

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
        vc->m_inv_I_b = R_b.transpose() * body_b->get_inv_inertia();
        vc->m_penetration = manifold->m_penetration;
        vc->m_contact_index = i;

        // the center of body in the world frame
        b3Transr xf_a(body_a->get_position(), body_a->get_quaternion());
        b3Transr xf_b(body_b->get_position(), body_b->get_quaternion());

        b3Vec3r center_a = xf_a.transform(body_a->get_local_center());
        b3Vec3r center_b = xf_b.transform(body_b->get_local_center());

        //////////////////////////// Position Constraints ////////////////////////////
        b3ContactPositionConstraint* pc = m_position_constraints + i;
        pc->m_index_a = vc->m_index_a;
        pc->m_index_b = vc->m_index_b;

        pc->m_inv_mass_a = body_a->get_inv_mass();
        pc->m_inv_mass_b = body_b->get_inv_mass();

        pc->m_center_a = center_a;
        pc->m_center_b = center_b;

        pc->m_inv_I_a = vc->m_inv_I_a;
        pc->m_inv_I_b = vc->m_inv_I_b;
        pc->m_local_normal = manifold->m_local_normal;
        pc->m_point_count = point_count;
        pc->m_local_point = manifold->m_local_point;
        pc->m_type = manifold->m_type;
        pc->m_radius_a = fixture_a->get_shape()->get_radius();
        pc->m_radius_b = fixture_b->get_shape()->get_radius();

        for (int32 j = 0; j < point_count; j++) {

            b3VelocityConstraintPoint* vcp = vc->m_points + j;
            b3ManifoldPoint* manifold_point = manifold->m_points + j;

            vcp->m_ra = manifold_point->m_local_point - center_a;
            vcp->m_rb = manifold_point->m_local_point - center_b;

            pc->m_local_points[j] = manifold_point->m_local_point;
        }
    }
}


void b3ContactSolver::init_velocity_constraints()
{
    for(int32 i = 0; i < m_count; ++i) {

        b3ContactVelocityConstraint* vc = m_velocity_constraints + i;
        b3ContactPositionConstraint* pc = m_position_constraints + i;
        b3Manifold* manifold = m_contacts[vc->m_contact_index]->get_manifold();
        int32 index_a = vc->m_index_a;
        int32 index_b = vc->m_index_b;

        b3Vec3r p_a = m_ps[index_a];
        b3Quatr q_a = m_qs[index_a];
        b3Vec3r v_a = m_vs[index_a];
        b3Vec3r w_a = m_ws[index_a];

        b3Vec3r p_b = m_ps[index_b];
        b3Quatr q_b = m_qs[index_b];
        b3Vec3r v_b = m_vs[index_b];
        b3Vec3r w_b = m_ws[index_b];

        b3Transr xf_a(p_a, q_a);
        b3Transr xf_b(p_b, q_b);

        real radius_a = pc->m_radius_a;
        real radius_b = pc->m_radius_b;

        b3WorldManifold world_manifold;
        world_manifold.initialize(manifold, xf_a, radius_a, xf_b, radius_b);

        vc->m_normal = world_manifold.normal;

        int32 point_count = vc->m_point_count;
        for (int32 j = 0; j < point_count; ++j) {
            b3VelocityConstraintPoint *vcp = vc->m_points + j;

            vcp->m_ra = world_manifold.points[j] - p_a;
            vcp->m_rb = world_manifold.points[j] - p_b;

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
            // m_bias_velocity is eJv
            vcp->m_bias_velocity = 0.0;
            if (v_rel < -vc->m_restitution_threshold) {
                vcp->m_bias_velocity = -vc->m_restitution * v_rel;
                int32 k = 0;
            }

            vcp->m_normal_impulse = 0.0;
            vcp->m_normal_contact_impulse = 0.0;
            vcp->m_tangent_impulse = 0.0;
        }
        vc->m_normal_contact_impulse = 0.0;
        vc->m_normal_collision_impulse = 0.0;

        if (point_count > 1 && g_block_solve) {

            b3Vec12r* J = (b3Vec12r*)m_block_allocator->allocate(point_count * sizeof(b3Vec12r));
            b3Vec12r* JW = (b3Vec12r*)m_block_allocator->allocate(point_count * sizeof(b3Vec12r));
            real* mem_JWJT = (real*)m_block_allocator->allocate(point_count * point_count * sizeof(real));
            vc->m_JWJT = (real**)m_block_allocator->allocate(point_count * sizeof(real*));

            // calculate JWJT
            b3Mat1212r W;
            W.set_zero();
            W.set_block(vc->m_inv_mass_a * b3Mat33r::identity(), 0, 0);
            W.set_block(vc->m_inv_I_a, 3, 3);
            W.set_block(vc->m_inv_mass_b * b3Mat33r::identity(), 6, 6);
            W.set_block(vc->m_inv_I_b, 9, 9);

            for (int32 i = 0; i < point_count; ++i) {
                vc->m_JWJT[i] = &mem_JWJT[i * point_count];
            }

            for (int32 i = 0; i < point_count; i++) {
                b3VelocityConstraintPoint* vcp = vc->m_points;
                J[i].set_segment(-vc->m_normal, 0);
                J[i].set_segment(-vcp[i].m_ra.cross(vc->m_normal), 3);
                J[i].set_segment(vc->m_normal, 6);
                J[i].set_segment(vcp[i].m_rb.cross(vc->m_normal), 9);
            }

            for (int32 i = 0; i < point_count; i++) {
                JW[i].set_zero();
                for (int32 j = 0; j < 12; j++) {
                    JW[i] += J[i][j] * W.row(j);
                }
            }

            for (int32 i = 0; i < point_count; ++i) {
                for (int32 j = 0; j < point_count; ++j) {
                    vc->m_JWJT[i][j] = JW[i].dot(J[j]);
                }
            }

            m_block_allocator->free(J, point_count * sizeof(b3Vec12r));
            m_block_allocator->free(JW, point_count * sizeof(b3Vec12r));
        }
    }
}


void b3ContactSolver::solve_velocity_constraints()
{
    for (int32 i = 0; i < m_count; ++i) {
        b3ContactVelocityConstraint *vc = m_velocity_constraints + i;

        const b3Vec3r& normal = vc->m_normal;

        b3Vec3r v_a = m_vs[vc->m_index_a];
        b3Vec3r w_a = m_ws[vc->m_index_a];

        b3Vec3r v_b = m_vs[vc->m_index_b];
        b3Vec3r w_b = m_ws[vc->m_index_b];

        if (vc->m_point_count == 1 || !g_block_solve) {

            for (int32 j = 0; j < vc->m_point_count; ++j) {
                b3VelocityConstraintPoint* vcp = vc->m_points + j;

                b3Vec3r v_rel = (v_b + w_b.cross(vcp->m_rb) - v_a - w_a.cross(vcp->m_ra));

                real vn = v_rel.dot(normal);

                real lambda = 0;

                lambda = -vcp->m_normal_mass * (vn - vcp->m_bias_velocity);
                real new_impulse = b3_max(vcp->m_normal_impulse + lambda, (real)0.0);
                lambda = new_impulse - vcp->m_normal_impulse;
                vcp->m_normal_impulse = new_impulse;

                // apply normal Impulse
                // (0,0, -1)
                // (0,0, -0.666)
                b3Vec3r impulse = lambda * normal;

                v_a = v_a - vc->m_inv_mass_a * impulse;
                w_a = w_a - vc->m_inv_I_a * vcp->m_ra.cross(impulse);
                v_b = v_b + vc->m_inv_mass_b * impulse;
                w_b = w_b + vc->m_inv_I_b * vcp->m_rb.cross(impulse);
            }

        } else {
            // this is a Linear Complementary Problem with max size of 16
            // vn = m_ * x + b, vn >= 0, xn >= 0, vni * xni = 0
            // m_ = J * M_inv * JT
            // b = vn0 - v_bias
            // because the system is iterative, use incremental impulse instead
            // x = a + d
            // x: new total impulse
            // d: incremental impulse
            // vn = M * d + b
            //    = M * (x - a) + b
            //    = M * x + b - M * a
            // vn = M * x + b'
            // b' = b - M * a
            b3Lemke lemke(m_block_allocator, vc, v_a, w_a, v_b, w_b);

            bool early_quit = lemke.initialize_problem();

            if (early_quit) {
                continue;
            }

            lemke.solve();

            for (int32 j = 0; j < vc->m_point_count; ++j) {
                b3VelocityConstraintPoint* vcp = vc->m_points + j;
                real lemke_impulse = lemke.get_normal_impulse(j);
                real inc_impulse = lemke_impulse - vcp->m_normal_impulse;

                b3Vec3r inc_impulse_v = inc_impulse * normal;
                spdlog::info("inc_impulse value: {}", inc_impulse);
                v_a = v_a - vc->m_inv_mass_a * inc_impulse_v;
                w_a = w_a - vc->m_inv_I_a * vcp->m_ra.cross(inc_impulse_v);
                v_b = v_b + vc->m_inv_mass_b * inc_impulse_v;
                w_b = w_b + vc->m_inv_I_b * vcp->m_rb.cross(inc_impulse_v);

                vcp->m_normal_impulse = vcp->m_normal_impulse + inc_impulse;
            }
        }
        m_vs[vc->m_index_a] = v_a;
        m_vs[vc->m_index_b] = v_b;
        m_ws[vc->m_index_a] = w_a;
        m_ws[vc->m_index_b] = w_b;
    }
}


struct b3PositionSolverManifold {

    void initialize(b3ContactPositionConstraint* pc, const b3Transr& xf_a, const b3Transr& xf_b, int32 index) {

        b3_assert(pc->m_point_count > 0);

        switch (pc->m_type)
        {
            case b3Manifold::e_spheres: {
                b3Vec3r point_a = xf_a.position();
                b3Vec3r point_b = xf_b.position();
                normal = pc->m_local_normal;
                point = 0.5 * (point_a + point_b);
                separation = (point_b - point_a).dot(normal) - pc->m_radius_a - pc->m_radius_b;
            }
            break;

            case b3Manifold::e_face_A:
            {
                normal = pc->m_local_normal;

                b3Vec3r plane_point = pc->m_local_point;

                b3Vec3 clipPoint = pc->m_local_points[index];
                separation = (clipPoint - plane_point).dot(normal) - pc->m_radius_a - pc->m_radius_b;
                point = clipPoint;
            }
            break;

            case b3Manifold::e_face_B:
            {
                normal = pc->m_local_normal;
                b3Vec3r plane_point = pc->m_local_point;

                b3Vec3 clipPoint = pc->m_local_points[index];
                separation = (clipPoint - plane_point).dot(normal) - pc->m_radius_a - pc->m_radius_b;
                point = clipPoint;
            }
            break;

            default:
                break;

        }
    }

    b3Vec3r normal;
    b3Vec3r point;
    float separation;
};


bool b3ContactSolver::solve_position_constraints()
{
    real min_separation = 0.0;

    for (int32 i = 0; i < m_count; ++i) {
        b3ContactPositionConstraint* pc = m_position_constraints + i;

        int32 index_a = pc->m_index_a;
        int32 index_b = pc->m_index_b;
        b3Vec3r center_a = pc->m_center_a;
        b3Vec3r center_b = pc->m_center_b;

        real m_a = pc->m_inv_mass_a;
        real m_b = pc->m_inv_mass_b;

        b3Mat33r I_a = pc->m_inv_I_a;
        b3Mat33r I_b = pc->m_inv_I_b;

        int32 point_count = pc->m_point_count;

        b3Vec3r p_a = m_ps[index_a];
        b3Quatr q_a = m_qs[index_a];

        b3Vec3r p_b = m_ps[index_b];
        b3Quatr q_b = m_qs[index_b];

        for (int32 j = 0; j < point_count; ++j) {
            b3Transr xf_a(p_a, q_a);
            b3Transr xf_b(p_b, q_b);

            b3PositionSolverManifold psm;
            psm.initialize(pc, xf_a, xf_b, j);
            b3Vec3r normal = psm.normal;
            b3Vec3r point = psm.point;
            real separation = psm.separation;

            b3Vec3r r_a = point - center_a;
            b3Vec3r r_b = point - center_b;

            min_separation = b3_min(min_separation, separation);

            // Prevent large corrections and allow slop.
            real C = b3_clamp(b3_baumgarte * (separation + b3_linear_slop), -b3_max_linear_correction, 0.0);

            b3Vec3r ra_n = r_a.cross(normal);
            b3Vec3r rb_n = r_b.cross(normal);

            real K = pc->m_inv_mass_a + pc->m_inv_mass_b +
                       (ra_n * pc->m_inv_I_a).dot(ra_n) +
                       (rb_n * pc->m_inv_I_b).dot(rb_n);

            real impulse = K > 0 ? -C / K : 0;

            b3Vec3 P = impulse * normal;

            p_a -= m_a * P;
            //q_a -= real(0.5) * b3Quatr(0, I_a * ra_n.cross(P)) * q_a;
            p_b += m_b * P;
            //q_b += real(0.5) * b3Quatr(0, I_b * rb_n.cross(P)) * q_b;
        }

        m_ps[index_a] = p_a;
        m_qs[index_a] = q_a;
        m_ps[index_b] = p_b;
        m_qs[index_b] = q_b;
    }
    return min_separation >= -3.0 * b3_linear_slop;
}


b3ContactSolver::~b3ContactSolver()
{
    for (int32 i = 0; i < m_count; ++i) {
        const int32& point_count = m_velocity_constraints[i].m_point_count;
        if (point_count > 1 && g_block_solve) {
            m_block_allocator->free(m_velocity_constraints[i].m_JWJT[0], point_count * point_count * sizeof(real));
            m_block_allocator->free(m_velocity_constraints[i].m_JWJT, point_count * sizeof(real*));
        }
    }

    m_block_allocator->free(m_velocity_constraints, m_count * sizeof(b3ContactVelocityConstraint));
    m_block_allocator->free(m_position_constraints, m_count * sizeof(b3ContactPositionConstraint));
}


