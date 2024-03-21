
#ifndef BOX3D_B3_SOLVER_FACTORY_HPP
#define BOX3D_B3_SOLVER_FACTORY_HPP

#include <variant>
#include <stdexcept>

#include "b3_solver.hpp"
#include "b3_si_solver.hpp"


enum b3SolverType {
    SI_SOLVER = 0,
    VELOCITY_SOLVER = 1
};

class b3SolverFactory {

public:

    static b3Solver* get_solver(b3SolverType type) {

        switch (type) {
            case SI_SOLVER:
                static b3SISolver si_solver;
                return &si_solver;
            default:
                throw std::invalid_argument("Invalid solver type");
        }
    }
};


#endif //BOX3D_B3_SOLVER_FACTORY_HPP
