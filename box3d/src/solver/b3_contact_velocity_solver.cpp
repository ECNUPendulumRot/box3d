
#include "solver/b3_contact_velocity_solver.hpp"
#include "collision/b3_fixture.hpp"
#include "collision/b3_contact.hpp"
#include "dynamics/b3_body.hpp"
#include "solver/b3_lemke.hpp"
#include "common/b3_time_step.hpp"
#include <spdlog/spdlog.h>

static bool g_block_solve = false;


b3ContactVelocitySolver::b3ContactVelocitySolver(b3ContactSolverDef *def)
{
    m_step = def->step;
    m_contacts = def->contacts;
    m_count = def->count;
    m_vel_iteration = def->vel_iteration;

    m_block_allocator = def->block_allocator;
    m_is_static_collide = def->is_static_collide;

    m_velocity_constraints = (b3ContactVelocityConstraint*)m_block_allocator->allocate(m_count * sizeof(b3ContactVelocityConstraint));

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
        vc->m_friction = contact->get_friction();
        vc->m_restitution = contact->get_restitution();
        vc->m_restitution_threshold = contact->get_restitution_threshold();
        vc->m_va = body_a->get_linear_velocity();
        vc->m_vb = body_b->get_linear_velocity();
        vc->m_wa = body_a->get_angular_velocity();
        vc->m_wb = body_b->get_angular_velocity();
        vc->m_pa = body_a->get_position();
        vc->m_pb = body_b->get_position();
        vc->m_qa = body_a->get_quaternion();
        vc->m_qb = body_b->get_quaternion();
        vc->m_radius_a = fixture_a->get_shape()->get_radius();
        vc->m_radius_b = fixture_b->get_shape()->get_radius();

        vc->m_inv_mass_a = body_a->get_inv_mass();
        vc->m_inv_mass_b = body_b->get_inv_mass();

        const b3Mat33r& R_a = body_a->get_quaternion().rotation_matrix();
        vc->m_inv_I_a = R_a.transpose() * body_a->get_inv_inertia() * R_a;
        const b3Mat33r& R_b = body_b->get_quaternion().rotation_matrix();
        vc->m_inv_I_b = R_b.transpose() * body_b->get_inv_inertia() * R_b;

        vc->m_point_count = point_count;
        vc->m_contact_index = i;
    }
}


void b3ContactVelocitySolver::init_velocity_constraints()
{

    for(int32 i = 0; i < m_count; ++i) {

        b3ContactVelocityConstraint* vc = m_velocity_constraints + i;

        b3Manifold* manifold = m_contacts[vc->m_contact_index]->get_manifold();

        const real& m_a = vc->m_inv_mass_a;
        const real& m_b = vc->m_inv_mass_b;
        const b3Mat33r& I_a = vc->m_inv_I_a;
        const b3Mat33r& I_b = vc->m_inv_I_b;

        b3Transr xf_a, xf_b;
        xf_a.set_quaternion(vc->m_qa);
        xf_b.set_quaternion(vc->m_qb);
        xf_a.set_position(vc->m_pa);
        xf_b.set_position(vc->m_pb);

        b3WorldManifold world_manifold;
        world_manifold.initialize(manifold, xf_a, vc->m_radius_a, xf_b, vc->m_radius_b);

        vc->m_normal = world_manifold.normal;

        int32 point_count = vc->m_point_count;
        for (int32 j = 0; j < point_count; ++j) {
            b3VelocityConstraintPoint *vcp = vc->m_points + j;

            vcp->m_ra = world_manifold.points[j] - vc->m_pa;
            vcp->m_rb = world_manifold.points[j] - vc->m_pb;

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
            real v_rel = vc->m_normal.dot(vc->m_vb + vc->m_wb.cross(vcp->m_rb) - vc->m_va - vc->m_wa.cross(vcp->m_ra));
            // m_bias_velocity is eJv
            vcp->m_bias_velocity = 0.0;
            if (v_rel < -vc->m_restitution_threshold) {
                vcp->m_bias_velocity = -vc->m_restitution * v_rel;
            }

            vcp->m_normal_impulse = 0.0;
        }

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


void b3ContactVelocitySolver::solve_velocity_constraints()
{
    for (int32 i = 0; i < m_vel_iteration; ++i) {

        for (int32 j = 0; j < m_count; ++j) {

            b3ContactVelocityConstraint *vc = m_velocity_constraints + j;

            const int32& index_a = vc->m_index_a;
            const int32& index_b = vc->m_index_b;
            const real& m_a = vc->m_inv_mass_a;
            const real& m_b = vc->m_inv_mass_b;
            const b3Mat33r& I_a = vc->m_inv_I_a;
            const b3Mat33r& I_b = vc->m_inv_I_b;
            const int32& point_count = vc->m_point_count;

            b3Vec3r v_a = vc->m_va;
            b3Vec3r w_a = vc->m_wa;
            b3Vec3r v_b = vc->m_vb;
            b3Vec3r w_b = vc->m_wb;
            const b3Vec3r& normal = vc->m_normal;

            for (int32 k = 0; k< vc->m_point_count; ++k) {
                b3VelocityConstraintPoint* vcp = vc->m_points + k;

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
    }
}


b3ContactVelocitySolver::~b3ContactVelocitySolver()
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



