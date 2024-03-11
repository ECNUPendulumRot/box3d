
#include "solver/b3_solver.hpp"

#include "dynamics/b3_island.hpp"
#include "dynamics/b3_body.hpp"
#include "collision/b3_collision.hpp"
#include "collision/b3_contact.hpp"
#include "common/b3_time_step.hpp"
#include "common/b3_block_allocator.hpp"
#include "collision/b3_fixture.hpp"

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


    /*
    // correct the body total force
    // m_contact link all bodies
    memory = m_block_allocator->allocate(sizeof(b3Body*) * m_body_count);
    b3Body** stack = new (memory) b3Body*;
    int index = 0;
//        if(m_bodies[i]->get_type() != b3BodyType::b3_static_body) {
//            continue;
//        }
    stack[index++] = m_bodies[0];
    m_bodies[0]->compute_total_force();
    // all bodies are in the island
    while(index > 0) {
        b3Body *b = stack[--index];
        b->set_flag(b3Body::e_contact_force_flag);
        // all direct or indirect contacts with static bodies all need modify the total force
        auto contacts = b->get_contact_list();
        while (contacts) {
            b3Body *other = contacts->m_other;
            if (other->test_flag(b3Body::e_contact_force_flag)) {
                contacts = contacts->m_next;
                continue;
            }
            stack[index++] = other;
            other->set_flag(b3Body::e_contact_force_flag);
            other->compute_total_force();
            // the contact normal is a point to b
            // this body is a or b?
            // so the normal is static body to this body
            b3Vector3d normal = contacts->m_contact->get_manifold()->m_local_normal;
            if (contacts->m_contact->get_fixture_a()->get_body() == other) {
                normal = -normal;
            }
            b3Vector3d other_force = other->get_total_force();
            double force_normal = other_force.dot(normal);
            if (force_normal < 0) {
                other_force -= force_normal * normal;
            }
            other->set_total_force(other_force);
            b3Vector3d b_force = b->get_total_force();
            force_normal = b_force.dot(normal);
            if (force_normal > 0) {
                b_force -= force_normal * normal;
            }
            b->set_total_force(b_force);

            contacts = contacts->m_next;
        }
    }

    // clear all flags
    for(int32 j = 0; j < m_body_count; ++j) {
        m_bodies[j]->unset_flag(b3Body::e_contact_force_flag);
    }

    m_block_allocator->free(stack, sizeof(b3Body*) * m_body_count);
*/

    for(int32 i = 0; i < m_body_count; ++i) {

        b3Body* b = m_bodies[i];
        m_positions[i] = b->get_pose();
        m_velocities[i] = b->get_velocity();

        // integrate velocity
        if(b->get_type() == b3BodyType::b3_dynamic_body) {
            b->compute_total_force();
            b3Vector3d v = m_velocities[i] .linear();
            b3Vector3d w = m_velocities[i] .angular();

            if(m_body_count == 1) {
                v += m_timestep->m_dt * b->get_inv_mass() * b->get_total_force();
                w += m_timestep->m_dt * b->get_inv_inertia() * b->get_torque();
            }

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
