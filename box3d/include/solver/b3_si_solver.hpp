
#ifndef BOX3D_B3_SI_SOLVER_HPP
#define BOX3D_B3_SI_SOLVER_HPP

#include "solver/b3_solver.hpp"

namespace box3d {
    class b3SISolver;
}


class box3d::b3SISolver : public box3d::b3Solver {

public:

    void initialize(b3World* world) {
        
    }

    void initialize(b3Island* island, b3TimeStep* timestep);

    void init_velocity_constraints();

    int solve();

    void solve_velocity_constraints();
};



#endif // BOX3D_B3_SI_SOLVER_HPP
