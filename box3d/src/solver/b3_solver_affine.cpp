
#include "solver/b3_solver_affine.hpp"

#include "dynamics/b3_world.hpp"

#include "common/b3_types.hpp"


void box3d::b3SolverAffine::initialize(box3d::b3World *world)
{
    m_world = world;
}


int box3d::b3SolverAffine::solve(double delta_t)
{
    m_body_list = (b3BodyAffine*)(m_world->m_affine_body_list);

    for (b3BodyAffine* body = m_body_list; body != nullptr; body = (b3BodyAffine*)body->next()) {

        // The affine gravity need to be revised
        b3Vector12d affine_gravity_acc = body->affine_gravity_acc(m_world->gravity());

        b3Vector12d n_p_grad = -body->get_potential_energy_gradient();

        b3Vector12d affine_q = body->get_q();

        b3Vector12d affine_q_dot = body->get_q_dot();

        b3Matrix12d affine_inv_mass = body->get_inv_mass_matrix();

        // semi-implicit Euler
        affine_q_dot += (affine_inv_mass * (n_p_grad) + affine_gravity_acc)* delta_t;
        affine_q += affine_q_dot * delta_t;

        body->set_q(affine_q);
        body->set_q_dot(affine_q_dot);
    }
    return 0;
}



