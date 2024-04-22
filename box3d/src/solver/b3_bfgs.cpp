
#include "solver/b3_bfgs.hpp"


namespace {

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

}


double b3BFGS::dist(
        const Eigen::Vector<double, 12> &q_a, const Eigen::Vector<double, 12> &q_b,
        const int32 &vc_i, const int32 &p_i) {

    const auto& Ja = m_vcs[vc_i].m_points[p_i].Ja;
    const auto& Jb = m_vcs[vc_i].m_points[p_i].Jb;

    double res = Ja.dot(q_a) + Jb.dot(q_b);
    return res;
}


double b3BFGS::ip_mass_terms() {

    double res = 0.0;

    for (int32 i = 0; i < m_body_count; ++i) {

        const Eigen::Vector<double, 12>& q_pred = m_q_preds[i];
        const Eigen::Matrix<double, 12, 12>& M = m_Ms[i];
        Eigen::Vector<double, 12> q_diff = m_q.segment<12>(i * 12) - q_pred;

        double m = 0.5 * q_diff.transpose() * M * q_diff;
        res += m;
    }
    return res;
}



double b3BFGS::ip_energy_terms() {

    const double* k = m_ks;
    const double* v = m_vs;
    const int32 body_count = m_body_count;

    double res = 0.0;
    for (int32 i = 0; i < body_count; ++i) {
        res += energy_term(k[i], v[i], m_q.segment<12>(i * 12));
    }
    return res;
}


double b3BFGS::ip_barrier_terms() {

    double res = 0.0;

    static double tr = 2 * (2.0 * b3_linear_slop);

    for (int32 i = 0; i < m_contact_count; i++) {

        const int32& point_count = m_vcs[i].m_point_count;

        int32 index_a = m_vcs[i].m_index_a;
        int32 index_b = m_vcs[i].m_index_b;

        const double& penetration = m_vcs[i].m_penetration - 0.0001;

        for (int32 j = 0; j < point_count; j++) {

            const auto& q_a = m_q.segment<12>(index_a * 12);
            const auto& q_b = m_q.segment<12>(index_b * 12);

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


Eigen::VectorXd b3BFGS::ip_mass_grad() {

    const int32 body_count = m_body_count;

    Eigen::VectorXd res = Eigen::VectorXd::Zero(m_q.size());

    for (int32 i = 0; i < body_count; ++i) {
        const Eigen::Vector<double, 12>& q_pred = m_q_preds[i];
        const Eigen::Matrix<double, 12, 12>& M = m_Ms[i];
        res.segment<12>(i * 12) = M * (m_q.segment<12>(i * 12) - q_pred);
    }

    return res;
}


Eigen::VectorXd b3BFGS::ip_energy_grad() {

    Eigen::VectorXd res = Eigen::VectorXd::Zero(m_q.size());

    for (int32 i = 0; i < m_body_count; ++i) {
        res.segment<12>(i * 12) = energy_gradient(m_ks[i], m_vs[i], m_q.segment<12>(i * 12));
    }

    return res;
}


Eigen::VectorXd b3BFGS::ip_barrier_grad() {
    Eigen::VectorXd res = Eigen::VectorXd::Zero(m_q.size());

    static double tr = 2 * (2.0 * b3_linear_slop);

    for (int32 i = 0; i < m_contact_count; i++) {

        const b3AffineContactVelocityConstraint* vc = m_vcs + i;
        const int32& point_count = vc->m_point_count;

        const int32& index_a = vc->m_index_a;
        const int32& index_b = vc->m_index_b;

        const double& p = vc->m_penetration - 0.00001;

        for (int32 j = 0; j < point_count; j++) {

            const b3VelocityConstraintPoint* vp = vc->m_points + j;

            const auto& q_a = m_q.segment<12>(index_a * 12);
            const auto& q_b = m_q.segment<12>(index_b * 12);


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


double b3BFGS::ip_fn(const Eigen::VectorXd &q, Eigen::VectorXd *grad_out, void *data) {

    m_q = q;

    double M_obj = ip_mass_terms();
    double V_obj = ip_energy_terms();
    double B_obj = m_bk * ip_barrier_terms();

    if (grad_out) {
        *grad_out = ip_mass_grad() + ip_energy_grad() + m_bk * ip_barrier_grad();
        //*grad_out = ip_mass_grad(q) + 100 * barrier_grad(q);
    }
    return M_obj + V_obj + B_obj;
}

