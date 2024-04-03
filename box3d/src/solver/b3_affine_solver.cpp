
#include "solver/b3_affine_solver.hpp"

#include "dynamics/b3_island.hpp"
#include "dynamics/b3_body.hpp"

#include "collision/b3_fixture.hpp"
#include "collision/b3_collision.hpp"
#include "collision/b3_contact.hpp"

#include "common/b3_time_step.hpp"

#include <spdlog/spdlog.h>

namespace {

    Eigen::Matrix<real, 3, 12> Jacobian(const b3Vector3r& v) {

        Eigen::Matrix<real, 3, 12> J;

        J.setZero();

        J(0, 0) = 1;
        J(1, 1) = 1;
        J(2, 2) = 1;

        J(0, 3)  = v.x();
        J(0, 4)  = v.y();
        J(0, 5)  = v.z();

        J(1, 6)  = v.x();
        J(1, 7)  = v.y();
        J(1, 8)  = v.z();

        J(2, 9)  = v.x();
        J(2, 10) = v.y();
        J(2, 11) = v.z();

        return J;
    }

    Eigen::Vector<real, 12> energy_gradient(const real& k, const real& v, const Eigen::Vector<real, 12>& affine_q) {

        Eigen::Vector<real, 12> result = Eigen::Vector<real, 12>::Zero();

        Eigen::Vector3<real> a1 = affine_q.segment<3>(3);
        Eigen::Vector3<real> a2 = affine_q.segment<3>(6);
        Eigen::Vector3<real> a3 = affine_q.segment<3>(9);

        Eigen::Matrix3<real> a1a1 = a1 * a1.transpose();
        Eigen::Matrix3<real> a2a2 = a2 * a2.transpose();
        Eigen::Matrix3<real> a3a3 = a3 * a3.transpose();

        result.segment<3>(3) = 2 * ((a1.dot(a1) - 1) * a1 + a2a2 * a1 + a3a3 * a1);
        result.segment<3>(6) = 2 * ((a2.dot(a2) - 1) * a2 + a1a1 * a2 + a3a3 * a2);
        result.segment<3>(9) = 2 * ((a3.dot(a3) - 1) * a3 + a1a1 * a3 + a2a2 * a3);

        return 2 * k * v * result;

    }

}


void b3AffineSolver::init(b3BlockAllocator *block_allocator, b3Island *island, b3TimeStep *step) {

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

        b3AffineContactVelocityConstraint* avc = m_avc + i;

        avc->m_contact_index = i;
        avc->m_point_count = point_count;

        avc->m_normal = {manifold->m_local_normal.x(), manifold->m_local_normal.y(), manifold->m_local_normal.z()};

        avc->m_index_a = body_a->get_island_index();
        avc->m_mass_a = body_a->get_mass();
        avc->m_inv_mass_a = body_a->get_inv_mass();


        avc->m_affine_I_a = body_a->get_affine_mass();
        avc->m_affine_inv_I_a = body_a->get_affine_inv_mass();

        avc->m_index_b = body_b->get_island_index();

        avc->m_mass_b = body_b->get_mass();
        avc->m_inv_mass_b = body_b->get_inv_mass();

        avc->m_affine_I_b = body_b->get_affine_mass();
        avc->m_affine_inv_I_b = body_b->get_affine_inv_mass();

        avc->m_penetration = manifold->m_penetration;

        avc->m_ra = b3Vector3r::zero();
        avc->m_rb = b3Vector3r::zero();

        // the center of body in the world frame
        b3Transformr xf_a(m_affine_qs[avc->m_index_a]);
        b3Transformr xf_b(m_affine_qs[avc->m_index_b]);

        for (int32 j = 0; j < point_count; j++) {
            b3VelocityConstraintPoint *vcp = avc->m_points + j;
            b3ManifoldPoint *manifold_point = manifold->m_points + j;

            // m_ra and m_rb are all in body's local frame
            vcp->m_ra = xf_a.transform_local(manifold_point->m_local_point);
            vcp->m_rb = xf_b.transform_local(manifold_point->m_local_point);
            // vcp->m_rhs_penetration = manifold->m_penetration;
            // TODO: warm start
        }
    }
}


void b3AffineSolver::init_velocity_constraints()
{
    for(int32 i = 0; i < m_contact_count; ++i) {

        b3AffineContactVelocityConstraint* avc = m_avc + i;
        int32 index_a = avc->m_index_a;
        int32 index_b = avc->m_index_b;

        Eigen::Vector<real, 12> q_a = m_affine_qs[index_a];
        Eigen::Vector<real, 12> q_b = m_affine_qs[index_b];

        Eigen::Vector<real, 12> q_dot_a = m_affine_q_dots[index_a];
        Eigen::Vector<real, 12> q_dot_b = m_affine_q_dots[index_b];

        for (int j = 0; j < avc->m_point_count; ++j) {

            b3VelocityConstraintPoint *vcp = avc->m_points + j;

            // vcp->m_ra(b) is a contact point(from the center of body)
            // vc->m_ra(b) is the average of all contact points
            avc->m_ra += vcp->m_ra;
            avc->m_rb += vcp->m_rb;

            Eigen::Vector3<real> normal = avc->m_normal;

            Eigen::Matrix<real, 3, 12> Jra = Jacobian(vcp->m_ra);
            Eigen::Matrix<real, 3, 12> Jrb = Jacobian(vcp->m_rb);

            Eigen::Vector<real, 12> jran = Jra.transpose() * normal;
            Eigen::Vector<real, 12> jrbn = Jrb.transpose() * normal;

            real jmj_a = jran.transpose() * avc->m_affine_inv_I_a * jran;
            real jmj_b = jrbn.transpose() * avc->m_affine_inv_I_b * jrbn;

            real jmj = jmj_a + jmj_b;

            // 1 / (J * M_inv * J^T)
            vcp->m_normal_mass = jmj > 0 ? real(1.0) / jmj : 0;

            // J * q-dot
            // real v_rel = normal.dot(Jrb * q_dot_b - Jra * q_dot_a);
            // m_rhs_restitution_velocity is eJv
            vcp->m_rhs_restitution_velocity = 0.0;

            vcp->m_normal_collision_impulse = 0.0;
            vcp->m_normal_contact_impulse = 0.0;
            vcp->m_tangent_impulse = 0.0;
        }
        avc->m_normal_contact_impulse = 0.0;
        avc->m_normal_collision_impulse = 0.0;
        avc->m_ra /= avc->m_point_count;
        avc->m_rb /= avc->m_point_count;

        // TODO: if we have more than one contact point, then prepare the block solver.
    }
}


int b3AffineSolver::solve()
{
    init_velocity_constraints();

    // collision
//    for(int32 i = 0; i < m_timestep->m_velocity_iterations; ++i) {
//        solve_velocity_constraints(true);
//    }
    // fix penetration
    // correct_penetration();

    // velocity update
    for(int32 i = 0; i < m_body_count; ++i) {

        b3Body *b = m_bodies[i];

        Eigen::Vector<real, 12> affine_q_dot = m_affine_q_dots[i];

        const real& k = b->get_stiffness();
        const real& v = b->get_volume();
        auto g = b->get_affine_gravity();
        auto M_inv = b->get_affine_inv_mass();
        auto d = energy_gradient(k, v, m_affine_qs[i]);
        auto f_d = b->get_affine_inv_mass() * d;
        affine_q_dot += m_timestep->m_dt * g - m_timestep->m_dt * M_inv * f_d;

        m_affine_q_dots[i] = affine_q_dot;
    }

    // collision
    for(int32 i = 0; i < m_timestep->m_velocity_iterations; ++i) {
        solve_velocity_constraints(true);
    }

    for (int32 i = 0; i < m_body_count; ++i) {
        m_affine_qs[i] = m_affine_qs[i] + m_timestep->m_dt * m_affine_q_dots[i];
    }

    // copy state buffers back to the bodies.
    write_states_back();

    return 0;
}


void b3AffineSolver::solve_velocity_constraints(bool is_collision)
{
    for (int32 i = 0; i < m_contact_count; ++i) {

        b3AffineContactVelocityConstraint *vc = m_avc + i;

        Eigen::Vector<real, 12> q_a = m_affine_qs[vc->m_index_a];
        Eigen::Vector<real, 12> q_b = m_affine_qs[vc->m_index_b];

        Eigen::Vector<real, 12> q_dot_a = m_affine_q_dots[vc->m_index_a];
        Eigen::Vector<real, 12> q_dot_b = m_affine_q_dots[vc->m_index_b];

        const Eigen::Vector3<real>& normal = vc->m_normal;

        for (int32 j = 0; j < vc->m_point_count; ++j) {

            b3VelocityConstraintPoint* vcp = vc->m_points + j;

            Eigen::Matrix<real, 3, 12> Jra = Jacobian(vcp->m_ra);
            Eigen::Matrix<real, 3, 12> Jrb = Jacobian(vcp->m_rb);

            real rhs = -(Jrb * q_dot_b - Jra * q_dot_a).dot(normal);

            real lambda;

            lambda = vcp->m_normal_mass * rhs;
            real new_impulse = b3_max(vcp->m_normal_collision_impulse + lambda, (real)0.0);
            lambda = new_impulse - vcp->m_normal_collision_impulse;
            vcp->m_normal_collision_impulse = new_impulse;

            // apply normal Impulse
            Eigen::Vector3<real> impulse = lambda * normal;

            Eigen::Vector<real, 12> delta_q_a = vc->m_affine_inv_I_a * Jra.transpose() * impulse;
            Eigen::Vector<real, 12> delta_q_b = vc->m_affine_inv_I_b * Jrb.transpose() * impulse;
            q_dot_a = q_dot_a - delta_q_a;
            q_dot_b = q_dot_b + delta_q_b;
        }
        m_affine_q_dots[vc->m_index_a] = q_dot_a;
        m_affine_q_dots[vc->m_index_b] = q_dot_b;

    }
}


