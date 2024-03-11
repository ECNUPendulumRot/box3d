
#include "solver/b3_solver.hpp"

#include "dynamics/b3_island.hpp"
#include "dynamics/b3_body.hpp"
#include "collision/b3_collision.hpp"
#include "collision/b3_contact.hpp"
#include "common/b3_time_step.hpp"
#include "common/b3_block_allocator.hpp"

b3Solver::b3Solver(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step) {
    m_timestep = step;
    m_block_allocator = block_allocator;

    m_contact_count = island->get_contacts_count();
    m_contacts = island->get_contacts();

    b3_assert(m_contact_count >= 0);

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

    memory = m_block_allocator->allocate(m_body_count * sizeof(b3TransformD));
    m_positions = new (memory) b3TransformD;

    memory = m_block_allocator->allocate(m_body_count * sizeof(b3TransformD));
    m_velocities = new (memory) b3TransformD;

    memory = m_block_allocator->allocate(m_body_count * sizeof(b3TransformD));
    m_velocities_last_time_step = new (memory) b3TransformD;

    for(int32 i = 0; i < m_body_count; ++i) {

        b3Body* b = m_bodies[i];
        m_positions[i] = b->get_pose();
        m_velocities[i] = b->get_velocity();
        m_velocities_last_time_step[i] = b->get_velocity();

        // integrate velocity 
        // To do:change it to verlet integrate
        if(b->get_type() == b3BodyType::b3_dynamic_body) {
            b3Vector3d v = m_velocities[i] .linear();
            b3Vector3d w = m_velocities[i] .angular();
            

            v += m_timestep->m_dt * b->get_inv_mass() * (b->get_force() + b->get_gravity());
            w += m_timestep->m_dt * b->get_inv_inertia() * b->get_torque();

            // TODO: apply damping

            m_velocities[i] .set_linear(v);
            m_velocities[i] .set_angular(w);
        }
    }
}


void b3Solver::write_states_back() {
    for(int32 i = 0; i < m_body_count; ++i) {
        b3Body* body = m_bodies[i];

        body->set_pose(m_positions[i]);
        body->set_velocity(m_velocities[i]);

        // TODO: if we need SynchronizeTransform() ?
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
