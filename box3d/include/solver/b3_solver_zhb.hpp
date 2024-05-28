
#ifndef BOX3D_B3_SOLVER_ZHB_HPP
#define BOX3D_B3_SOLVER_ZHB_HPP

#include "dynamics/b3_transform.hpp"
#include "math/b3_quat.hpp"
#include "solver/b3_contact_constraint.hpp"
#include "common/b3_time_step.hpp"

/////////// Forward Delaration ///////////

class b3TimeStep;

class b3Contact;

class b3World;

class b3Body;

class b3Island;

class b3ContactVelocityConstraint;

class b3BlockAllocator;

//////////////////////////////////////////


class b3SolverZHB {

protected:

    b3Contact** m_contacts = nullptr;
    int32 m_contact_count;

    int32 m_body_count;

    b3Vec3r* m_ps = nullptr;

    b3Quatr* m_qs = nullptr;

    b3Vec3r* m_vs = nullptr;

    b3Vec3r* m_ws = nullptr;

    b3ContactVelocityConstraint* m_velocity_constraints = nullptr;

    b3TimeStep* m_timestep = nullptr;

    b3BlockAllocator* m_block_allocator = nullptr;

    b3Body** m_bodies;

    b3IntegralMethod m_method;

    int32* m_wait = nullptr;

    /**
     * @brief write the velocity and position back to bodies.
     */
    void write_states_back();

public:

    b3SolverZHB() = default;

    b3SolverZHB(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step);

    void init(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step);

    void solve_velocity_constraints(bool is_collision);

    void init_velocity_constraints();

    int solve(bool allow_sleep);

    ~b3SolverZHB();

    void clear();
};




#endif 
