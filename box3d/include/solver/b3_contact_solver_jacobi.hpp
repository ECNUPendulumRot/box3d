# pragma once


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

struct b3JacobiSim {

    b3BodySim* body_sim;

    b3Vec3r v;
    b3Vec3r w;
    b3Vec3r p;
    b3Quatr q;

};


struct b3ContactSimJacobi: b3ContactSim {
    b3Vec3r delta_v_a = b3Vec3r(0, 0, 0);
    b3Vec3r delta_w_a = b3Vec3r(0, 0, 0);
    b3Vec3r delta_v_b = b3Vec3r(0, 0, 0);
    b3Vec3r delta_w_b = b3Vec3r(0, 0, 0);
};


class b3ContactSolverJacobi {

public:

    b3Contact** m_contacts = nullptr;

    int32 m_contact_count;

    b3TimeStep* m_timestep = nullptr;

    b3BlockAllocator* m_block_allocator = nullptr;

    b3Body** m_bodies;

    bool m_is_static = false;

    b3Island* m_island = nullptr;

    b3ContactSimJacobi* m_contact_constraints = nullptr;

    b3ContactSolverJacobi() = default;

    b3ContactSolverJacobi(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step);

    void init(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step);

    void prepare_contact_contraints();

    void solve_velocity_constraints();

    ~b3ContactSolverJacobi();

    bool empty_solver() const {
        return m_contact_count == 0;
    }

};
