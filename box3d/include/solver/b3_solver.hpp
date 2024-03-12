
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


class b3Solver {

protected:

    b3Contact** m_contacts = nullptr;
    int32 m_contact_count;

    int32 m_body_count;
    b3TransformD* m_positions = nullptr;
    b3TransformD* m_velocities = nullptr;
    b3TransformD* m_velocities_w_f = nullptr;//used in verlet integration, store the velocity with out force applied

    b3ContactVelocityConstraint* m_velocity_constraints = nullptr;

    b3TimeStep* m_timestep = nullptr;

    b3BlockAllocator* m_block_allocator = nullptr;

    b3Body** m_bodies;

    void write_states_back();

    void correct_force();

public:

    b3Solver() = delete;

    b3Solver(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step);

    // virtual void initialize(b3World* world) = 0;

    //virtual int solve() = 0;

    virtual int solve(int type) = 0;

    ~b3Solver();

};




#endif //BOX3D_B3_SOLVER_HPP