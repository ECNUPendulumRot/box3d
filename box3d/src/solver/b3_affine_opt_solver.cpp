
#include "solver/b3_affine_opt_solver.hpp"

#include "dynamics/b3_island.hpp"
#include "dynamics/b3_body.hpp"

#include "collision/b3_fixture.hpp"
#include "collision/b3_collision.hpp"
#include "collision/b3_contact.hpp"

#include "common/b3_time_step.hpp"

#include <spdlog/spdlog.h>

#include "utils/b3_timer.hpp"
#include "solver/b3_bfgs.hpp"

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


    inline double dist(const Eigen::Vector<double, 12>& q_a, const Eigen::Vector<double, 12>& q_b, const int32& vc_i, const int32& p_i) {

        const auto& Ja = opt_data.vcs[vc_i].m_points[p_i].Ja;
        const auto& Jb = opt_data.vcs[vc_i].m_points[p_i].Jb;

        double res = Ja.dot(q_a) + Jb.dot(q_b);
        return res;
    }

    double ip_mass_terms(const Eigen::VectorXd& q) {
        double res = 0.0;

        const Eigen::Vector<double, 12>* q_pred_in = opt_data.q_pred;
        const Eigen::Matrix<double, 12, 12>* M_in = opt_data.Ms;
        const int32 body_count = opt_data.body_count;

        for (int32 i = 0; i < body_count; ++i) {

            const Eigen::Vector<double, 12>& q_pred = q_pred_in[i];
            const Eigen::Matrix<double, 12, 12>& M = M_in[i];
            Eigen::Vector<double, 12> q_diff = q.segment<12>(i * 12) - q_pred;

            double m = 0.5 * q_diff.transpose() * M * q_diff;
            res += m;
        }
        return res;
    }
    
    Eigen::VectorXd ip_mass_grad(const Eigen::VectorXd& q) {

        const Eigen::Vector<double, 12>* q_pred_in = opt_data.q_pred;
        const Eigen::Matrix<double, 12, 12>* M_in = opt_data.Ms;
        const int32 body_count = opt_data.body_count;

        Eigen::VectorXd res = Eigen::VectorXd::Zero(q.size());

        for (int32 i = 0; i < body_count; ++i) {
            const Eigen::Vector<double, 12>& q_pred = q_pred_in[i];
            const Eigen::Matrix<double, 12, 12>& M = M_in[i];
            res.segment<12>(i * 12) = M * (q.segment<12>(i * 12) - q_pred);
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

    Eigen::VectorXd ip_energy_grad(const Eigen::VectorXd& q) {
        const int32 body_count = opt_data.body_count;
        const double* k = opt_data.ks;
        const double* v = opt_data.vs;


        Eigen::VectorXd res = Eigen::VectorXd::Zero(q.size());

        for (int32 i = 0; i < body_count; ++i) {
            res.segment<12>(i * 12) = energy_gradient(k[i], v[i], q.segment<12>(i * 12));
        }

        return res;
    }

    double barrier_terms(const Eigen::VectorXd& q) {

        const b3AffineContactVelocityConstraint* vcs = opt_data.vcs;
        const int32& contact_count = opt_data.contact_count;

        double res = 0.0;

        static double tr = 2 * (2.0 * b3_linear_slop);

        for (int32 i = 0; i < contact_count; i++) {

            const int32& point_count = vcs[i].m_point_count;

            int32 index_a = vcs[i].m_index_a;
            int32 index_b = vcs[i].m_index_b;

            const double& penetration = vcs[i].m_penetration - 0.0001;

            for (int32 j = 0; j < point_count; j++) {

                const auto& q_a = q.segment<12>(index_a * 12);
                const auto& q_b = q.segment<12>(index_b * 12);

                double d = dist(q_a, q_b, i, j);

                if (d >= 0) {
                    continue;
                }

                double s = (d - penetration) / b3_abs(penetration);
                double b_value = - d * d * log(s);

                res += b_value;
            }
        }
        return res;
    }

    Eigen::VectorXd barrier_grad(const Eigen::VectorXd& q) {

        Eigen::VectorXd res = Eigen::VectorXd::Zero(q.size());

        static double tr = 2 * (2.0 * b3_linear_slop);

        for (int32 i = 0; i < opt_data.contact_count; i++) {

            const b3AffineContactVelocityConstraint* vc = opt_data.vcs + i;
            const int32& point_count = vc->m_point_count;

            const int32& index_a = vc->m_index_a;
            const int32& index_b = vc->m_index_b;

            const double& p = vc->m_penetration - 0.00001;

            for (int32 j = 0; j < point_count; j++) {

                const b3VelocityConstraintPoint* vp = vc->m_points + j;

                const auto& q_a = q.segment<12>(index_a * 12);
                const auto& q_b = q.segment<12>(index_b * 12);


                double d = dist(q_a, q_b, i, j);


                if (d >= 0) {
                    continue;
                }

                double s = (d - p) / b3_abs(p);
                double coeff = -d * d / (d - p) - 2 * d * log(s);


                res.segment<12>(index_a * 12) += coeff * vp->Ja;
                res.segment<12>(index_b * 12) += coeff * vp->Jb;

            }
        }

        return res;
    }


    double ip_fn(const Eigen::VectorXd& q, Eigen::VectorXd* grad_out, void* data) {

        double M_obj = ip_mass_terms(q);
        double V_obj = ip_energy_terms(q);
        double B_obj = 10 * barrier_terms(q);
        if (grad_out) {
            *grad_out = ip_mass_grad(q) + ip_energy_grad(q) + 10 * barrier_grad(q);
            //*grad_out = ip_mass_grad(q) + 100 * barrier_grad(q);
        }
        return M_obj + V_obj + B_obj;
    };
}


void b3AffineOptSolver::init(b3BlockAllocator *block_allocator, b3Island *island, b3TimeStep *step) {

    b3Solver::init(block_allocator, island, step);

    m_constraint_count = 0;

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

        avc->m_type_a = body_a->get_type();
        avc->m_type_b = body_b->get_type();

        if (manifold->flipped)
            avc->flip();

        avc->m_penetration = manifold->m_penetration;

        // the center of body in the world frame
        b3Transformr xf_a(m_affine_qs[avc->m_index_a]);
        b3Transformr xf_b(m_affine_qs[avc->m_index_b]);

        for (int32 j = 0; j < point_count; j++) {
            b3VelocityConstraintPoint *vcp = avc->m_points + j;
            b3ManifoldPoint *manifold_point = manifold->m_points + j;

            vcp->m_ra = xf_a.transform_local(manifold_point->m_local_point);
            vcp->m_rb = xf_b.transform_local(manifold->m_local_point);

            m_constraint_count++;
        }
    }
}


void b3AffineOptSolver::init_collision_constraints() {

    for(int32 i = 0; i < m_contact_count; ++i) {

        b3AffineContactVelocityConstraint* avc = m_avc + i;

        for (int j = 0; j < avc->m_point_count; ++j) {

            b3VelocityConstraintPoint *vcp = avc->m_points + j;

            Eigen::Vector3<double> normal = avc->m_normal.cast<double>();

            if (avc->m_type_a != b3BodyType::b3_static_body) {
                Eigen::Matrix<double, 3, 12> Jra = Jacobian(vcp->m_ra);
                vcp->Ja = Jra.transpose() * normal;
            }

            if (avc->m_type_b != b3BodyType::b3_static_body) {
                Eigen::Matrix<double, 3, 12> Jrb = Jacobian(vcp->m_rb);
                vcp->Jb = -Jrb.transpose() * normal;
            }
        }
    }
}


int b3AffineOptSolver::solve()
{

    ///
    spdlog::log(spdlog::level::info, "------------------ Solving Affine Constraints------------------");
    const auto& delta_t = m_timestep->m_dt;

    init_collision_constraints();

    b3BFGS bfgs = b3BFGS();
    bfgs.m_delta_t = delta_t;
    bfgs.m_ks = m_ks;
    bfgs.m_vs = m_vs;
    bfgs.m_Ms = m_Ms;
    bfgs.m_bodies = m_bodies;
    bfgs.m_vcs = m_avc;
    bfgs.m_q_preds = m_affine_q_preds;
    bfgs.m_body_count = m_body_count;
    bfgs.m_contact_count = m_contact_count;
    bfgs.m_q.resize(m_body_count * 12);
    bfgs.m_q.setZero();

    bool success = bfgs.solve(m_affine_qs);
    // b3_assert(success == true);

    for(int32 i = 0; i < m_body_count; ++i) {

        Eigen::Vector<real, 12> q = bfgs.m_q.segment<12>(12 * i).cast<real>();
        Eigen::Vector<real, 12> q_dot = (q - m_affine_qs[i]) / delta_t;
        m_affine_qs[i] = q;
        m_affine_q_dots[i] = q_dot;
    }

    write_states_back();

    return 0;
}






