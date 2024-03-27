
#include "solver/b3_solver.hpp"

#include "dynamics/b3_island.hpp"
#include "dynamics/b3_body.hpp"
#include "collision/b3_collision.hpp"
#include "collision/b3_contact.hpp"
#include "common/b3_time_step.hpp"
#include "common/b3_block_allocator.hpp"
#include "collision/b3_fixture.hpp"


//////////////////////////////////////////
// In the tangent plane, t1 and t2 are counterclockwise
/*
 *   ^ t2
 *   |
 *   |
 *   n----> t1
 *
 */
void b3_get_two_tangent_bases(const b3Vector3r& normal, b3Vector3r& t1, b3Vector3r& t2)
{
    if(b3_abs(normal.x()) < b3_real_min && b3_abs(normal.y()) < b3_real_min) {
        t1 = b3Vector3r(1, 0, 0);
        t2 = b3Vector3r(0 , 1, 0);
        return;
    }
    t1 = b3Vector3r(-normal.y(), normal.x(), 0);
    t2 = normal.cross(t1).normalized();
    t1 = t2.cross(normal).normalized();
}

/////////////////////////////////////////

//b3Solver::b3Solver(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step)
//{
//  init(block_allocator, island, step);
//}


void b3Solver::init(b3BlockAllocator *block_allocator, b3Island *island, b3TimeStep *step)
{
    m_method = step->m_integral_method;
    m_timestep = step;
    m_block_allocator = block_allocator;

    m_contact_count = island->get_contacts_count();
    m_contacts = island->get_contacts();

    b3_assert(m_contact_count >= 0);

    // allocate memory for all bodies and contacts of every island.
    // and we need extra arrays to store the velocity and position of every body.
    // after solver all contacts and friction constraints, we copy the results back to bodies.

    // The number of velocity constraints is same to the number of contacts.
    void* memory;
    if(m_contact_count > 0) {
        memory = m_block_allocator->allocate(m_contact_count * sizeof(b3ContactVelocityConstraint));
        m_velocity_constraints = new (memory) b3ContactVelocityConstraint;
    } else {
        m_velocity_constraints = nullptr;
    }

    m_body_count = island->get_body_count();
    b3_assert(m_body_count > 0);
    m_bodies = island->get_bodies();

    memory = m_block_allocator->allocate(m_body_count * sizeof(b3Transformr));
    m_positions = new (memory) b3Transformr;

    memory = m_block_allocator->allocate(m_body_count * sizeof(b3Transformr));
    m_velocities = new (memory) b3Transformr;

    memory = m_block_allocator->allocate(m_body_count * sizeof(b3Quaternionr));
    m_qs = new (memory) b3Quaternionr;

    // TODO: check if delete this.
    memory = m_block_allocator->allocate(m_body_count * sizeof(b3Transformr));
    m_velocities_w_f = new (memory) b3Transformr;

    memory = m_block_allocator->allocate(m_body_count * sizeof(b3Vector3r));
    m_vs = new (memory) b3Vector3r;

    memory = m_block_allocator->allocate(m_body_count * sizeof(b3Vector3r));
    m_ws = new (memory) b3Vector3r;

    for(int32 i = 0; i < m_body_count; ++i) {
        b3Body* b = m_bodies[i];
        m_positions[i] = b->get_pose();
        m_velocities[i] = b->get_velocity();
        m_qs[i] = b->get_quaternion();
        m_vs[i] = b->get_linear_velocity();
        m_ws[i] = b->get_angular_velocity();
    }
}


void b3Solver::write_states_back()
{
    for(int32 i = 0; i < m_body_count; ++i) {
        m_bodies[i]->set_pose(m_positions[i]);
        m_bodies[i]->set_velocity(m_velocities[i]);
        m_bodies[i]->set_linear_velocity(m_vs[i]);
        m_bodies[i]->set_angular_velocity(m_ws[i]);
        // TODO: if we need SynchronizeTransform() ?
    }
}


b3Solver::~b3Solver()
{
    clear();
}

void b3Solver::clear() {
    m_timestep = nullptr;
    m_block_allocator->free(m_contacts, m_contact_count * sizeof(b3Contact*));
    m_block_allocator->free(m_velocity_constraints, m_contact_count * sizeof(b3ContactVelocityConstraint));
    m_block_allocator->free(m_positions, m_body_count * sizeof(b3Transformr));
    m_block_allocator->free(m_velocities, m_body_count * sizeof(b3Transformr));
    // TODO: check if delete this
    m_block_allocator->free(m_velocities_w_f, m_body_count * sizeof(b3Transformr));
    m_block_allocator->free(m_qs, m_body_count * sizeof(b3Quaternionr));
    m_block_allocator->free(m_vs, m_body_count * sizeof(b3Vector3r));
    m_block_allocator->free(m_ws, m_body_count * sizeof(b3Vector3r));
    m_block_allocator = nullptr;
}


