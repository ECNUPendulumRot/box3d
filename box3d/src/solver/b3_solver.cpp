
#include "solver/b3_solver.hpp"

#include "dynamics/b3_island.hpp"
#include "dynamics/b3_body.hpp"


#include "collision/b3_fixture.hpp"
#include "collision/b3_contact.hpp"

#include "common/b3_time_step.hpp"
#include "common/b3_block_allocator.hpp"

#include "solver/b3_contact_constraint.hpp"
#include "solver/b3_lemke.hpp"

#include "spdlog/spdlog.h"
#include "solver/b3_contact_solver.hpp"


bool g_block_solve = false;

b3Solver::b3Solver(b3BlockAllocator *block_allocator, b3Island *island, b3TimeStep *step)
{
    init(block_allocator, island, step);
}


void b3Solver::init(b3BlockAllocator *block_allocator, b3Island *island, b3TimeStep *step)
{
    m_timestep = step;
    m_block_allocator = block_allocator;

    m_contact_count = island->get_contacts_count();
    m_contacts = island->get_contacts();

    b3_assert(m_contact_count >= 0);

    // allocate memory for all bodies and contacts of every island.
    // and we need extra arrays to store the velocity and position of every body.
    // after solver all contacts and friction constraints, we copy the results back to bodies.

    // The number of velocity constraints is same to the number of contacts.
    m_body_count = island->get_body_count();
    b3_assert(m_body_count > 0);

    m_ps = (b3Vec3r*)m_block_allocator->allocate(m_body_count * sizeof(b3Vec3r));
    m_qs = (b3Quatr*)m_block_allocator->allocate(m_body_count * sizeof(b3Quatr));
    m_vs = (b3Vec3r*)m_block_allocator->allocate(m_body_count * sizeof(b3Vec3r));
    m_ws = (b3Vec3r*)m_block_allocator->allocate(m_body_count * sizeof(b3Vec3r));

    m_bodies = island->get_bodies();
    for(int32 i = 0; i < m_body_count; ++i) {
        b3Body* b = m_bodies[i];
        m_ps[i] = b->get_position();
        m_qs[i] = b->get_quaternion();
        m_vs[i] = b->get_linear_velocity();
        m_ws[i] = b->get_angular_velocity();
    }
    ////////////////////////// Initialize Contact Constraints //////////////////////////
}


void b3Solver::write_states_back()
{
    for(int32 i = 0; i < m_body_count; ++i) {
        b3Body* b = m_bodies[i];
        b->m_sweep.p = m_ps[i];
        b->m_sweep.q = m_qs[i];

        b->set_linear_velocity(m_vs[i]);
        b->set_angular_velocity(m_ws[i]);

        b->set_position(m_ps[i]);
        b->set_quaternion(m_qs[i]);
    }
}


int b3Solver::solve(bool allow_sleep)
{
    // spdlog::info("|||||||||||||||||| Solve ||||||||||||||||||");

    for(int32 i = 0; i < m_body_count; ++i) {

        b3Body* b = m_bodies[i];

        b3Vec3r v = m_vs[i];
        b3Vec3r w = m_ws[i];

        b3Vec3r p = b->m_sweep.p;
        b3Quatr q = b->m_sweep.q;

        b->m_sweep.p0 = p;
        b->m_sweep.q0 = q;

        v += m_timestep->m_dt * b->get_inv_mass() * (b->get_force() + b->get_gravity());
        w += m_timestep->m_dt * b->get_inv_inertia() * b->get_torque();

        m_vs[i] = v;
        m_ws[i] = w;
        m_ps[i] = p;
        m_qs[i] = q;
    }

    b3ContactSolverDef def;
    def.step = *m_timestep;
    def.contacts = m_contacts;
    def.count = m_contact_count;
    def.ps = m_ps;
    def.qs = m_qs;
    def.vs = m_vs;
    def.ws = m_ws;
    def.block_allocator = m_block_allocator;

    b3ContactSolver contact_solver(&def);
    contact_solver.init_velocity_constraints();

    for(int32 i = 0; i < m_timestep->m_velocity_iterations; ++i) {
        contact_solver.solve_velocity_constraints();
    }



    for (int32 i = 0; i < m_body_count; ++i) {
        m_ps[i] = m_ps[i] + m_vs[i] * m_timestep->m_dt;
        m_qs[i] = m_qs[i] + real(0.5) * m_timestep->m_dt * b3Quatr(0, m_ws[i]) * m_qs[i];
        m_qs[i].normalize();
    }

    // integrate positions and rotations.
    bool position_solved = false;
    for (int32 i = 0; i < m_timestep->m_position_iterations; ++i) {
        bool contact_ok = contact_solver.solve_position_constraints();

        if (contact_ok) {
            position_solved = true;
            break;
        }
    }

    // copy state buffers back to the bodies.
    write_states_back();
    allow_sleep = false;
    if (allow_sleep) {
        const float lin_tor_sqr = b3_linear_sleep_tolerance * b3_linear_sleep_tolerance;
        const float ang_tor_sqr = b3_angular_sleep_tolerance * b3_angular_sleep_tolerance;

        for (int32 i = 0; i < m_body_count; ++i) {
            b3Body* b = m_bodies[i];
            if (b->get_type() == b3BodyType::b3_static_body) {
                continue;
            }

            b3Vec3r lin_vel = b->get_linear_velocity();
            b3Vec3r ang_vel = b->get_angular_velocity();

            if (lin_vel.dot(lin_vel) < lin_tor_sqr && ang_vel.dot(ang_vel) < ang_tor_sqr) {
                b->set_awake(false);
            } else {
                b->set_awake(true);
            }
        }
    }

    return 0;
}


b3Solver::~b3Solver()
{
    m_block_allocator->free(m_ps, m_body_count * sizeof(b3Vec3r));
    m_block_allocator->free(m_qs, m_body_count * sizeof(b3Quatr));
    m_block_allocator->free(m_vs, m_body_count * sizeof(b3Vec3r));
    m_block_allocator->free(m_ws, m_body_count * sizeof(b3Vec3r));
}


