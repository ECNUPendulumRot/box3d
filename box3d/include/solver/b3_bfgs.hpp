
#ifndef BOX3D_B3_BFGS_HPP
#define BOX3D_B3_BFGS_HPP

#include <Eigen/Dense>
#include "collision/b3_contact.hpp"
#include "solver/b3_contact_constraint.hpp"


class b3BFGS {

    Eigen::VectorXd q;

    Eigen::Matrix<double, 12, 12>* Ms;

    Eigen::Vector<double, 12>* q_pred;

    b3AffineContactVelocityConstraint* vcs;

    b3Body** bodies;

    double* ks;

    double* vs;

    double delta_t;

    int32 body_count;

    int32 contact_count;

public:

private:

    double ip_mass_terms();

    double ip_energy_terms();

    double ip_barrier_terms();

//    double ip_mass_grad();
//
//    double ip_energy_grad();
//
//    double ip_barrier_grad();


    double dist(const Eigen::Vector<double, 12>& q_a, const Eigen::Vector<double, 12>& q_b, const int32& vc_i, const int32& p_i);
};


#endif //BOX3D_B3_BFGS_HPP
