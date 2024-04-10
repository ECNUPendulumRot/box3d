
#ifndef BOX3D_B3_AFFINE_OPT_SOLVER_HPP
#define BOX3D_B3_AFFINE_OPT_SOLVER_HPP


#include "solver/b3_solver.hpp"

#include <Eigen/Dense>

#define OPTIM_ENABLE_EIGEN_WRAPPERS
#include "optim/optim.hpp"



class b3AffineOptSolver : public b3Solver {

    Eigen::VectorXd m_affine_lambda;

public:

    b3AffineOptSolver() = default;

    void init(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step) override;

    void init_collision_constraints();

    int solve() override;

};


#endif //BOX3D_B3_AFFINE_OPT_SOLVER_HPP
