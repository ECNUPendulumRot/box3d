
#ifndef BOX3D_B3_SI_SOLVER_HPP
#define BOX3D_B3_SI_SOLVER_HPP

#include "solver/b3_solver.hpp"


class b3SISolver : public b3Solver {

public:

    b3SISolver() = delete;

    b3SISolver(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step);

    void init_velocity_constraints();

    // type 0 = semi-implict Euler integration
    // type 1 = verlet integration
    int solve(int type) override;

    void solve_velocity_constraints(bool is_collision);

    void solve_friction_constraints();

    void correct_penetration();

    ~b3SISolver() = default;

    void solve_friction_help(int index_a, int index_b, const b3Vector3r &normal, const b3Vector3r &tangent,
                             real friction_impulse, real support_impulse, const b3Vector3r &ra, const b3Vector3r &rb,
                             b3Vector3r *points_ra, b3Vector3r *points_rb, int point_count,
                             b3ContactVelocityConstraint* vc);
};



#endif // BOX3D_B3_SI_SOLVER_HPP
