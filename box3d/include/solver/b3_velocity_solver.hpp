
#ifndef BOX3D_B3_VELOCITY_SOLVER_HPP
#define BOX3D_B3_VELOCITY_SOLVER_HPP


#include "solver/b3_solver.hpp"


class b3VelocitySolver : public b3Solver {

public:

    b3VelocitySolver() = default;

    //b3SISolver(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step);

    void init(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step) override;

    void init_velocity_constraints();

    int solve() override;

    void solve_velocity_constraints(bool is_collision);

    void correct_penetration();

    ~b3VelocitySolver() override = default;

};


#endif //BOX3D_B3_VELOCITY_SOLVER_HPP
