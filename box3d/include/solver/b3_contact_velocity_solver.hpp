#pragma once

#include "common/b3_time_step.hpp"
#include "collision/b3_collision.hpp"
#include "solver/b3_contact_constraint.hpp"

class b3BlockAllocator;
class b3Contact;


struct b3ContactSolverDef
{
    b3TimeStep step;
    b3Contact** contacts;
    real dt_substep;
    int32 vel_iteration;
    int32 count;
    b3Vec3r* ps;
    b3Quatr* qs;
    b3Vec3r* vs;
    b3Vec3r* ws;
    bool is_static_collide;
    b3BlockAllocator* block_allocator;
};


class b3ContactVelocitySolver {

    b3Contact** m_contacts = nullptr;
    int32 m_count;

    b3TimeStep m_step;

    int32 m_vel_iteration = 0;

    int32 m_substep = 4;

    bool m_is_static_collide = false;

    b3BlockAllocator* m_block_allocator = nullptr;

    b3ContactPositionConstraint* m_position_constraints;
    b3ContactVelocityConstraint* m_velocity_constraints;

public:

    b3ContactVelocitySolver(b3ContactSolverDef* def);

    ~b3ContactVelocitySolver();

    void init_velocity_constraints();

    void solve_velocity_constraints();
};


