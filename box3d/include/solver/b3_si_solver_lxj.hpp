
#ifndef BOX3D_B3_SI_SOLVER_LXJ_HPP
#define BOX3D_B3_SI_SOLVER_LXJ_HPP

#include "solver/b3_solver.hpp"


class b3SISolverLxj : public b3Solver {

public:

    b3SISolverLxj() = default;

    //b3SISolver(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step);

    void init(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step) override;

    void init_velocity_constraints();

    int solve() override;

    void solve_velocity_constraints(bool is_collision);

    void solve_friction_constraints();

    void correct_penetration();

    ~b3SISolverLxj() = default;
};



#endif // BOX3D_B3_SI_SOLVER_HPP
