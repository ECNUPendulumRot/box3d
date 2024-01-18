
#ifndef BOX3D_B3_SI_SOLVER_HPP
#define BOX3D_B3_SI_SOLVER_HPP

#include "solver/b3_solver.hpp"


class b3SISolver : public b3Solver {

public:

    void initialize(b3World* world) {
        
    }

    void initialize(b3Island* island, b3TimeStep* timestep);

    void init_velocity_constraints();

    int solve();

    void solve_velocity_constraints();
};



#endif // BOX3D_B3_SI_SOLVER_HPP
