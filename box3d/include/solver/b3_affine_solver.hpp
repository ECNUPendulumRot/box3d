
#ifndef BOX3D_B3_AFFINE_SOLVER_HPP
#define BOX3D_B3_AFFINE_SOLVER_HPP

#include "solver/b3_solver.hpp"

#include <Eigen/Dense>

struct b3AffineContactVelocityConstraint {

    b3VelocityConstraintPoint m_points[4];
    b3Vector3r m_normal;
    int32 m_index_a;
    int32 m_index_b;
    real m_inv_mass_a;
    real m_inv_mass_b;
    real m_mass_a;
    real m_mass_b;

    Eigen::Matrix<real, 12, 12> m_affine_I_a;
    Eigen::Matrix<real, 12, 12> m_affine_I_b;
    Eigen::Matrix<real, 12, 12> m_affine_inv_I_a;
    Eigen::Matrix<real, 12, 12> m_affine_inv_I_b;

    int32 m_point_count;
    int32 m_contact_index;

    real m_restitution;

    real m_penetration;
    // TODO
    real m_normal_collision_impulse = 0;
    real m_normal_contact_impulse = 0;

    b3Vector3r m_ra;
    b3Vector3r m_rb;

    real m_friction;
};


class b3AffineSolver : public b3Solver {

    b3AffineContactVelocityConstraint* m_avc = nullptr;

    Eigen::Vector<real, 12>* m_affine_qs = nullptr;

    Eigen::Vector<real, 12>* m_affine_q_dots = nullptr;

    void write_states_back() override;

public:

    b3AffineSolver() = default;

    //b3SISolver(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step);

    void init(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step) override;

    void init_velocity_constraints();

    int solve() override;

    void solve_velocity_constraints(bool is_collision);

    void correct_penetration();

    ~b3AffineSolver() override;

};


#endif //BOX3D_B3_AFFINE_SOLVER_HPP
