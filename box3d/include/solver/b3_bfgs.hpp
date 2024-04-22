
#ifndef BOX3D_B3_BFGS_HPP
#define BOX3D_B3_BFGS_HPP

#include <Eigen/Dense>
#include "collision/b3_contact.hpp"
#include "solver/b3_contact_constraint.hpp"


class b3BFGS {

public:

    Eigen::VectorXd m_q;

    Eigen::Matrix<double, 12, 12>* m_Ms;

    Eigen::Vector<double, 12>* m_q_preds;

    b3AffineContactVelocityConstraint* m_vcs;

    b3Body** m_bodies;

    double* m_ks;

    double* m_vs;

    double m_delta_t;

    int32 m_body_count;

    int32 m_contact_count;

    double m_bk = 1000000;

public:

    double ip_fn(const Eigen::VectorXd& q, Eigen::VectorXd* grad_out, void* data);

private:

    double ip_mass_terms();

    double ip_energy_terms();

    double ip_barrier_terms();

    Eigen::VectorXd ip_mass_grad();

    Eigen::VectorXd ip_energy_grad();

    Eigen::VectorXd ip_barrier_grad();

    double dist(const Eigen::Vector<double, 12>& q_a, const Eigen::Vector<double, 12>& q_b, const int32& vc_i, const int32& p_i);
    
};


#endif //BOX3D_B3_BFGS_HPP
