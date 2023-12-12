
#ifndef BOX3D_B3_AFFINE_SOLVER_HPP
#define BOX3D_B3_AFFINE_SOLVER_HPP


#include "solver/b3_solver.hpp"

#include "dynamics/b3_body_affine.hpp"

#include "common/b3_types.hpp"


namespace box3d {

    class b3AffineSolver;

}


class box3d::b3AffineSolver: public b3Solver {

    b3BodyAffine* m_body_list;

public:

    int solve() override;

    static b3Vector12d get_potential_energy_gradient(b3BodyAffine* body);

private:

    static b3Vector12d orthogonal_potential_gradient(b3BodyAffine* body);

};


#endif //BOX3D_B3_AFFINE_SOLVER_HPP
