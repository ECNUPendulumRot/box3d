
#ifndef BOX3D_B3_SOLVER_FACTORY_HPP
#define BOX3D_B3_SOLVER_FACTORY_HPP

#include <variant>
#include <stdexcept>

#include "b3_solver.hpp"
#include "b3_affine_solver.hpp"

enum b3SolverType {
    SI_SOLVER = 0,
    VELOCITY_SOLVER = 1,
    AFFINE_SOLVER = 2
};

class b3SolverFactory {

public:

    static b3Solver* get_solver(b3SolverType type) {
        static b3AffineSolver affine_solver;
        switch (type) {
            case AFFINE_SOLVER:
                return &affine_solver;

            default:
                return nullptr;
        }
    }
};


#endif //BOX3D_B3_SOLVER_FACTORY_HPP
