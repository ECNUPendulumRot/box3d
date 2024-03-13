
#ifndef BOX3D_B3_SI_SOLVER_HPP
#define BOX3D_B3_SI_SOLVER_HPP

#include "solver/b3_solver.hpp"


class b3SISolver : public b3Solver {

public:

    b3SISolver() = delete;

    b3SISolver(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step);

//    void initialize(b3World* world) { }

    void init_velocity_constraints();

    int solve() override;

    void solve_velocity_constraints(bool is_collision);

    void solve_friction_constraints();

    void correct_penetration();

    ~b3SISolver() = default;
};



#endif // BOX3D_B3_SI_SOLVER_HPP
