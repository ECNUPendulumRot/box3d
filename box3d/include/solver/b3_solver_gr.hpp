
#ifndef BOX3D_B3_SOLVER_GR_HPP
#define BOX3D_B3_SOLVER_GR_HPP


#include "dynamics/b3_transform.hpp"
#include "math/b3_quat.hpp"


/////////// Forward Delaration ///////////

class b3TimeStep;

class b3Contact;

class b3World;

class b3Body;

class b3Island;

class b3ContactVelocityConstraint;

class b3BlockAllocator;

//////////////////////////////////////////

class b3SolverGR {

    b3Contact** m_contacts = nullptr;

    int32 m_contact_count;

    int32 m_constraint_count = 0;

    int32 m_body_count;

    b3Vec3r* m_ps = nullptr;

    b3Quaternionr* m_qs = nullptr;

    b3Vec3r* m_vs = nullptr;

    b3Vec3r* m_ws = nullptr;

    b3ContactVelocityConstraint* m_velocity_constraints = nullptr;

    b3ContactVelocityConstraint** m_violated_constraints = nullptr;
    int32 m_violated_count = 0;

    b3TimeStep* m_timestep = nullptr;

    b3BlockAllocator* m_block_allocator = nullptr;

    b3Body** m_bodies;

public:

    b3SolverGR() = default;

    b3SolverGR(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step);

    void init(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step);

    void solve_velocity_constraints(int32 velocity_iterations);

    void init_velocity_constraints();

    int solve(bool allow_sleep);

    void write_states_back();

    ~b3SolverGR();

    void find_violated_constraints();
};


#endif //BOX3D_B3_SOLVER_GR_HPP