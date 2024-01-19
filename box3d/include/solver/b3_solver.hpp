
#ifndef BOX3D_B3_SOLVER_HPP
#define BOX3D_B3_SOLVER_HPP

#include "dynamics/b3_transform.hpp"

#include "common/b3_allocator.hpp"


/////////// Forward Delaration ///////////

class b3TimeStep;

class b3Contact;

class b3World;

class b3Island;

class b3ContactVelocityConstraint;

//////////////////////////////////////////


class b3Solver {

protected:

    b3Contact** m_contacts = nullptr;
	int32 m_contact_count;

    int32 m_body_count;
    b3TransformD* m_positions = nullptr;
    b3TransformD* m_velocities = nullptr;

    b3ContactVelocityConstraint* m_velocity_constraints = nullptr;

    b3TimeStep* m_timestep;

public:

    virtual void initialize(b3World* world) = 0;

    virtual void initialize(b3Island* island, b3TimeStep* step) = 0;

    virtual int solve() = 0;

    virtual ~b3Solver() {
        b3_free(m_contacts);
        b3_free(m_positions);
        b3_free(m_velocities);
        b3_free(m_velocity_constraints);
    }
    
};

#endif //BOX3D_B3_SOLVER_HPP
