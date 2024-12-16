
#ifndef BOX3D_B3_SOLVER_SUBSTEP_HPP
#define BOX3D_B3_SOLVER_SUBSTEP_HPP


#include "b3_contact_constraint.hpp"
#include "collision/b3_collision.hpp"
#include "dynamics/b3_island.hpp"
#include "dynamics/b3_transform.hpp"
#include "math/b3_quat.hpp"


/////////// Forward Delaration ///////////

class b3TimeStep;

class b3Contact;

class b3World;

class b3Body;

class b3Island;

class b3ContactVelocityConstraint;
class b3ContactPositionConstraint;

class b3BlockAllocator;

//////////////////////////////////////////


class b3ContactSolver {

public:

    b3Contact** m_contacts = nullptr;

    int32 m_contact_count;

    b3TimeStep* m_timestep = nullptr;

    b3BlockAllocator* m_block_allocator = nullptr;

    b3Body** m_bodies;

    bool m_is_static = false;

    b3Island* m_island = nullptr;

    b3ContactSim* m_contact_constraints = nullptr;

    b3ContactSolver() = default;

    b3ContactSolver(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step, bool is_static);

    void init(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step, bool is_static);

    void prepare_contact_contraints();

    void solve_velocity_constraints();

    ~b3ContactSolver();

    bool empty_solver() const {
        return m_contact_count == 0;
    }

};


#endif //BOX3D_B3_SOLVER_HPP