
#ifndef BOX3D_B3_SI_SOLVER_HPP
#define BOX3D_B3_SI_SOLVER_HPP

#include "solver/b3_solver.hpp"


class b3SISolver : public b3Solver {

public:

    b3SISolver() = delete;

    b3SISolver(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step);

//    void initialize(b3World* world) { }

    void init_velocity_constraints();

    // type 0 = semi-implict Euler integration
    // type 1 = verlet integration
    int solve(int type);

    void solve_velocity_constraints(bool is_collision);

    void correct_penetration();
};



#endif // BOX3D_B3_SI_SOLVER_HPP
