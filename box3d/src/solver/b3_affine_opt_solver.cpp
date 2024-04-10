
#include "solver/b3_affine_opt_solver.hpp"

#include "dynamics/b3_island.hpp"
#include "dynamics/b3_body.hpp"

#include "collision/b3_fixture.hpp"
#include "collision/b3_collision.hpp"
#include "collision/b3_contact.hpp"

#include "common/b3_time_step.hpp"

#include <spdlog/spdlog.h>

#include "utils/b3_timer.hpp"


namespace {

    Eigen::Matrix<double, 3, 12> Jacobian(const b3Vector3r& v) {

        Eigen::Matrix<double, 3, 12> J;

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

    Eigen::Vector<double, 12> energy_gradient(const double& k, const double& v, const Eigen::Vector<double, 12>& affine_q) {

        Eigen::Vector<double, 12> result = Eigen::Vector<double, 12>::Zero();

        Eigen::Vector3d a1 = affine_q.segment<3>(3);
        Eigen::Vector3d a2 = affine_q.segment<3>(6);
        Eigen::Vector3d a3 = affine_q.segment<3>(9);

        Eigen::Matrix3d a1a1 = a1 * a1.transpose();
        Eigen::Matrix3d a2a2 = a2 * a2.transpose();
        Eigen::Matrix3d a3a3 = a3 * a3.transpose();

        result.segment<3>(3) = 2 * ((a1.dot(a1) - 1) * a1 + a2a2 * a1 + a3a3 * a1);
        result.segment<3>(6) = 2 * ((a2.dot(a2) - 1) * a2 + a1a1 * a2 + a3a3 * a2);
        result.segment<3>(9) = 2 * ((a3.dot(a3) - 1) * a3 + a1a1 * a3 + a2a2 * a3);

        return 2.0 * k * v * result;

    }

    struct OptData {
        Eigen::Matrix<double, 12, 12>* Ms;
        Eigen::Vector<double, 12>* q_pred;
        b3AffineContactVelocityConstraint* vcs;
        b3Body** bodies;
        double* ks;
        double* vs;
        double delta_t;
        int32 body_count;
        int32 contact_count;
    } opt_data;


    double ip_mass_terms(const Eigen::VectorXd& q) {
        double res = 0.0;

        const Eigen::Vector<double, 12>* q_pred_in = opt_data.q_pred;
        const Eigen::Matrix<double, 12, 12>* M_in = opt_data.Ms;
        const int32 body_count = opt_data.body_count;

        for (int32 i = 0; i < body_count; ++i) {

            const Eigen::Vector<double, 12>& q_pred = q_pred_in[i];
            const Eigen::Matrix<double, 12, 12>& M = M_in[i];
            Eigen::Vector<double, 12> q_diff = q.segment<12>(i * 12) - q_pred;

            res += 0.5 * q_diff.transpose() * M * q_diff;
        }
        return res;
    }

    double energy_term(const double& k, const double& v, const Eigen::Vector<double, 12>& q) {
        double V_obj = 0.0;

        const Eigen::Vector3d& a1 = q.segment<3>(3);
        const Eigen::Vector3d& a2 = q.segment<3>(6);
        const Eigen::Vector3d& a3 = q.segment<3>(9);

        V_obj = (a1.dot(a1) - 1) * (a1.dot(a1) - 1)
                + (a2.dot(a2) - 1) * (a2.dot(a2) - 1)
                + (a3.dot(a3) - 1) * (a3.dot(a3) - 1);
        V_obj += 2 * (a1.dot(a2) * a1.dot(a2)
                      + a2.dot(a3) * a2.dot(a3)
                      + a3.dot(a1) * a3.dot(a1));

        return k * v * V_obj;
    }

    double ip_energy_terms(const Eigen::VectorXd& q) {
        const double* k = opt_data.ks;
        const double* v = opt_data.vs;
        const int32 body_count = opt_data.body_count;

        double res = 0.0;
        for (int32 i = 0; i < body_count; ++i) {
            res += energy_term(k[i], v[i], q.segment<12>(i * 12));
        }
        return res;
    }

    // TODO: check this function
    Eigen::VectorXd ip_mass_grad(const Eigen::VectorXd& q) {

        const Eigen::Vector<double, 12>* q_pred_in = opt_data.q_pred;
        const Eigen::Matrix<double, 12, 12>* M_in = opt_data.Ms;
        const int32 body_count = opt_data.body_count;

        Eigen::VectorXd res = Eigen::VectorXd::Zero(q.size());

        for (int32 i = 0; i < body_count; ++i) {
            const Eigen::Vector<double, 12>& q_pred = q_pred_in[i];
            const Eigen::Matrix<double, 12, 12>& M = M_in[i];
            res.segment<12>(i * 12) = q.segment<12>(i * 12) - q_pred;
        }

        return res;
    }

    Eigen::VectorXd ip_energy_grad(const Eigen::VectorXd& q) {
        const int32 body_count = opt_data.body_count;
        const double* k = opt_data.ks;
        const double* v = opt_data.vs;


        Eigen::VectorXd res = Eigen::VectorXd::Zero(q.size());

        for (int32 i = 0; i < body_count; ++i) {
            res.segment<12>(i * 12) += energy_gradient(k[i], v[i], q.segment<12>(i * 12));
        }

        return res;
    }

    double ip_fn(const Eigen::VectorXd& q, Eigen::VectorXd* grad_out, void* opt_data) {

        OptData* data = static_cast<OptData*>(opt_data);

        double M_obj = ip_mass_terms(q);
        double V_obj = ip_energy_terms(q);

        if (grad_out) {
            *grad_out = ip_mass_grad(q) + ip_energy_grad(q);
        }
        return M_obj + V_obj;
    };
}


void b3AffineOptSolver::init(b3BlockAllocator *block_allocator, b3Island *island, b3TimeStep *step) {

    b3Solver::init(block_allocator, island, step);

    int32 constraint_count = 0;

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

            constraint_count++;
        }
    }
    // The number of constraints is the number of contacts.
    m_affine_lambda.resize(constraint_count);
    m_affine_lambda.setZero();
}


void b3AffineOptSolver::init_collision_constraints() {

    for(int32 i = 0; i < m_contact_count; ++i) {

        b3AffineContactVelocityConstraint* avc = m_avc + i;

        for (int j = 0; j < avc->m_point_count; ++j) {

            b3VelocityConstraintPoint *vcp = avc->m_points + j;

            avc->m_ra += vcp->m_ra;
            avc->m_rb += vcp->m_rb;

            Eigen::Vector3<double> normal = avc->m_normal.cast<double>();

            Eigen::Matrix<double, 3, 12> Jra = Jacobian(vcp->m_ra);
            Eigen::Matrix<double, 3, 12> Jrb = Jacobian(vcp->m_rb);

            vcp->Ja = -Jra.transpose() * normal;
            vcp->Jb = Jrb.transpose() * normal;
        }
        avc->m_normal_contact_impulse = 0.0;
        avc->m_normal_collision_impulse = 0.0;
        avc->m_ra /= avc->m_point_count;
        avc->m_rb /= avc->m_point_count;
    }
}


int b3AffineOptSolver::solve()
{
    // velocity update
    const auto& delta_t = m_timestep->m_dt;

    init_collision_constraints();

    opt_data.delta_t = delta_t;
    opt_data.ks = m_ks;
    opt_data.vs = m_vs;
    opt_data.Ms = m_Ms;
    opt_data.bodies = m_bodies;
    opt_data.vcs = m_avc;
    opt_data.q_pred = m_affine_q_preds;
    opt_data.body_count = m_body_count;
    opt_data.contact_count = m_contact_count;

    Eigen::VectorXd affine_qs;
    affine_qs.resize(m_body_count * 12 + m_contact_count);
    affine_qs.setZero();
    for(int32 i = 0; i < m_body_count; ++i) {
        affine_qs.segment<12>(i * 12) = m_affine_qs[i].cast<double>();
    }

    bool success = optim::bfgs(affine_qs, ip_fn, &opt_data);

    b3_assert(success == true);

    for(int32 i = 0; i < m_body_count; ++i) {

        Eigen::Vector<real, 12> q = affine_qs.segment<12>(12 * i).cast<real>();
        Eigen::Vector<real, 12> q_dot = (q - m_affine_qs[i]) / delta_t;
        m_affine_qs[i] = q;
        m_affine_q_dots[i] = q_dot;
    }

    write_states_back();

    return 0;
}






