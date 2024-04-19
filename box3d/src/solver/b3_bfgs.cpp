
#include "solver/b3_bfgs.hpp"


double b3BFGS::dist(
        const Eigen::Vector<double, 12> &q_a, const Eigen::Vector<double, 12> &q_b,
        const int32 &vc_i, const int32 &p_i) {
    const auto& Ja = vcs[vc_i].m_points[p_i].Ja;
    const auto& Jb = vcs[vc_i].m_points[p_i].Jb;

    double res = Ja.dot(q_a) + Jb.dot(q_b);
    return res;
}


double b3BFGS::ip_mass_terms() {

    double res = 0.0;

    const Eigen::Vector<double, 12>* q_pred_in = q_pred;
    const Eigen::Matrix<double, 12, 12>* M_in = Ms;

    for (int32 i = 0; i < body_count; ++i) {

        const Eigen::Vector<double, 12>& q_pred = q_pred_in[i];
        const Eigen::Matrix<double, 12, 12>& M = M_in[i];
        Eigen::Vector<double, 12> q_diff = q.segment<12>(i * 12) - q_pred;

        double m = 0.5 * q_diff.transpose() * M * q_diff;
        res += m;
    }
    return res;
}


double b3BFGS::ip_energy_terms() {

    const int32 body_count = body_count;

    double res = 0.0;

    for (int32 i = 0; i < body_count; ++i) {

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

        res += ks[i] * vs[i] * V_obj;
    }
    return res;
}


double b3BFGS::ip_barrier_terms() {

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

            if (d >= tr) {
                continue;
            }

            double s = 1.0 / (d - penetration);
            double b_value = (d - tr) * (d - tr) * s * s;

            res += b_value;
        }
    }
    return res;
}

