
#include "solver/b3_solver.hpp"

#include "solver/b3_solver_substep_split_island.hpp"


b3Solver* b3Solver::get_solver(uint32 solver_type, b3World* world, b3BlockAllocator* block_allocator) {

    switch (solver_type) {
        case e_substep_split_island:
            return b3SolverSubstepSplitIsland::create(world, block_allocator);
        default:
            return b3Solver::create(world, block_allocator);
    }
}
