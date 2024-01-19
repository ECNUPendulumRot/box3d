
#include "solver/b3_solver.hpp"

#include "dynamics/b3_island.hpp"
#include "dynamics/b3_body.hpp"
#include "collision/b3_collision.hpp"
#include "collision/b3_contact.hpp"
#include "common/b3_time_step.hpp"
#include "common/b3_block_allocator.hpp"


b3Solver::b3Solver(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step) {
    m_timestep = step;

    m_contact_count = island->get_contacts_count();
    m_contacts = island->get_contacts();

    void* memory = m_block_allocator->allocate(m_contact_count * sizeof(b3ContactVelocityConstraint));
    m_velocity_constraints = new (memory) b3ContactVelocityConstraint;

    m_body_count = island->get_body_count();
    b3Body** bodies = island->get_bodies();

    memory = m_block_allocator->allocate(m_body_count * sizeof(b3TransformD));
    m_positions = new (memory) b3TransformD;

    memory = m_block_allocator->allocate(m_body_count * sizeof(b3TransformD));
    m_velocities = new (memory) b3TransformD;

    for(int32 i = 0; i < m_body_count; ++i) {
        m_positions[i] = bodies[i]->get_pose();
        m_velocities[i] = bodies[i]->get_velocity();
    }
}


b3Solver::~b3Solver() {
    m_timestep = nullptr;

    m_block_allocator->free(m_contacts, m_contact_count * sizeof(b3Contact*));
    m_block_allocator->free(m_velocity_constraints, m_contact_count * sizeof(b3ContactVelocityConstraint));
    m_block_allocator->free(m_positions, m_body_count * sizeof(b3TransformD));
    m_block_allocator->free(m_velocities, m_body_count * sizeof(b3TransformD));

    m_block_allocator = nullptr;
}

