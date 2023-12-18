
#ifndef BOX3D_B3_SOLVER_AFFINE_HPP
#define BOX3D_B3_SOLVER_AFFINE_HPP


#include "solver/b3_solver.hpp"

#include "dynamics/b3_body_affine.hpp"


namespace box3d {

    class b3SolverAffine;

    /////////////////////////

    class b3World;

}


class box3d::b3SolverAffine: public b3Solver {

    b3BodyAffine* m_body_list = nullptr;

    b3World* m_world = nullptr;

public:

    void initialize(b3World* world) override;

    int solve(double delta_t) override;

};


#endif //BOX3D_B3_SOLVER_AFFINE_HPP
