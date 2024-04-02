
#ifndef BOX3D_B3_AFFINE_SOLVER_HPP
#define BOX3D_B3_AFFINE_SOLVER_HPP

#include "solver/b3_solver.hpp"

#include <Eigen/Dense>


class b3AffineSolver : public b3Solver {

public:

    b3AffineSolver() = default;

    //b3SISolver(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step);

    void init(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step) override;

    void init_velocity_constraints();

    int solve() override;

    void solve_velocity_constraints(bool is_collision);

};


#endif //BOX3D_B3_AFFINE_SOLVER_HPP
