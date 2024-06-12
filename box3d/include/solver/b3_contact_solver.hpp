
#ifndef BOX3D_B3_CONTACT_SOLVER_HPP
#define BOX3D_B3_CONTACT_SOLVER_HPP


#include "common/b3_time_step.hpp"
#include "collision/b3_collision.hpp"
#include "solver/b3_contact_constraint.hpp"

class b3BlockAllocator;
class b3Contact;


struct b3ContactSolverDef
{
    b3TimeStep step;
    b3Contact** contacts;
    int32 count;
    b3Vec3r* ps;
    b3Quatr* qs;
    b3Vec3r* vs;
    b3Vec3r* ws;
    b3BlockAllocator* block_allocator;
};


class b3ContactSolver {

    b3Vec3r* m_ps = nullptr;

    b3Quatr* m_qs = nullptr;

    b3Vec3r* m_vs = nullptr;

    b3Vec3r* m_ws = nullptr;

    b3Contact** m_contacts = nullptr;
    int32 m_count;

    b3TimeStep m_step;

    b3BlockAllocator* m_block_allocator = nullptr;

    b3ContactPositionConstraint* m_position_constraints;
    b3ContactVelocityConstraint* m_velocity_constraints;

public:

    b3ContactSolver(b3ContactSolverDef* def);

    ~b3ContactSolver();

    void init_velocity_constraints();

    void solve_velocity_constraints();

    bool solve_position_constraints();
};


#endif //BOX3D_B3_CONTACT_SOLVER_HPP
