
#ifndef BOX3D_B3_SOLVER_HPP
#define BOX3D_B3_SOLVER_HPP


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


class b3Solver {

    b3Contact** m_contacts = nullptr;

    int32 m_contact_count;

    int32 m_body_count;

    b3Vec3r* m_ps = nullptr;

    b3Quaternionr* m_qs = nullptr;

    b3Vec3r* m_vs = nullptr;

    b3Vec3r* m_ws = nullptr;

    b3ContactVelocityConstraint* m_velocity_constraints = nullptr;

    b3TimeStep* m_timestep = nullptr;

    b3BlockAllocator* m_block_allocator = nullptr;

    b3Body** m_bodies;

    uint8* m_delayed;

public:

    b3Solver() = default;

    b3Solver(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step);

    void init(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step);

    void solve_velocity_constraints();

    void init_velocity_constraints();

    int solve();

    void write_states_back();

    ~b3Solver();

};




#endif //BOX3D_B3_SOLVER_HPP