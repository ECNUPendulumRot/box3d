
#ifndef BOX3D_B3_SOLVER_HPP
#define BOX3D_B3_SOLVER_HPP

#include "dynamics/b3_transform.hpp"

#include "solver/b3_contact_constraint.hpp"

/////////// Forward Delaration ///////////

class b3TimeStep;

class b3Contact;

class b3World;

class b3Body;

class b3Island;

class b3ContactVelocityConstraint;

class b3BlockAllocator;

//////////////////////////////////////////

void b3_get_two_tangent_bases(const b3Vector3d& normal, b3Vector3d& t1, b3Vector3d& t2);


class b3Solver {

protected:

    b3Contact** m_contacts = nullptr;
    int32 m_contact_count;

    int32 m_body_count;
    b3TransformD* m_positions = nullptr;
    b3TransformD* m_velocities = nullptr;
    //used in verlet integration, store the velocity with out force applied
    b3TransformD* m_velocities_w_f = nullptr;

    b3ContactVelocityConstraint* m_velocity_constraints = nullptr;

    b3TimeStep* m_timestep = nullptr;

    b3BlockAllocator* m_block_allocator = nullptr;

    b3Body** m_bodies;

    /**
     * @brief write the velocity and position back to bodies.
     */
    void write_states_back();

public:

    b3Solver() = delete;

    b3Solver(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step);


    /**
     * @breif solver all constraints in the island.
     */
    virtual int solve(int type) = 0;

    virtual ~b3Solver();
};




#endif //BOX3D_B3_SOLVER_HPP