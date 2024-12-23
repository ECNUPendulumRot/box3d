
#pragma once

#include "b3_contact_solver_jacobi.hpp"
#include "b3_contact_velocity_solver.hpp"
#include "b3_solver.hpp"
#include "dynamics/b3_island.hpp"
#include "collision/b3_contact.hpp"

struct b3SolverJacobi: b3Solver {

    int32 m_body_count;
    b3ContactManager* m_contact_manager;
    b3Body* m_body_list;
    b3Vec3r m_gravity;


    b3SolverJacobi(b3World* world, b3BlockAllocator* block_allocator):b3Solver(world, block_allocator) {

        m_body_count = world->get_body_count();
        m_contact_manager = &world->m_contact_manager;
        m_body_list = world->m_body_list;
        m_gravity = world->m_gravity;
    }

    void destroy() override {
        this->~b3SolverJacobi();
        m_block_allocator->free(this, sizeof(b3SolverJacobi));
    }

    void solve(b3TimeStep &step, b3BodySim *body_sims) override {

        b3Island island(m_block_allocator, m_body_count, m_contact_manager->m_contact_count);

        void* mem = m_block_allocator->allocate(m_body_count * sizeof(b3Body *));
        b3Body **stack = new (mem) b3Body *;
        for (b3Body *body = m_body_list; body; body = body->next()) {

            if (body->m_flags & b3Body::e_island_flag) {
                continue;
            }

            if (body->get_type() == b3BodyType::b3_static_body) {
                continue;
            }

            int32 stack_count = 0;
            stack[stack_count++] = body;
            body->m_flags |= b3Body::e_island_flag;

            // Perform a depth first search (DFS) on the constraint graph.
            while (stack_count > 0) {
                b3Body *b = stack[--stack_count];

                island.add_body(b->m_body_sim);

                // do not propagate islands across static bodies
                if (b->get_type() == b3BodyType::b3_static_body) {
                    continue;
                }

                // search all contact connected to this body
                for (b3ContactEdge *ce = b->get_contact_list(); ce; ce = ce->m_next) {
                    b3Contact *contact = ce->m_contact;

                    // Has this contact already has been added to this island ?
                    if (contact->test_flag(b3Contact::e_island_flag)) {
                        continue;
                    }
                    if (!contact->test_flag(b3Contact::e_touching_flag)) {
                        continue;
                    }

                    island.add_contact(contact);
                    contact->set_flag(b3Contact::e_island_flag);

                    b3Body *other = ce->m_other;

                    // Was the other body already has been added to this island ?
                    if (other->m_flags & b3Body::e_island_flag) {
                        continue;
                    }

                    stack[stack_count++] = other;
                    other->m_flags |= b3Body::e_island_flag;
                }
            }


            integrate_velocity(island.m_bodies, island.m_body_count, step.m_dt);

            // solve the constraints
            // b3Solver solver(&m_block_allocator, island, &step);
            b3ContactSolverJacobi solver(m_block_allocator, &island, &step);

            solver.prepare_contact_contraints();
            solver.solve_velocity_constraints();

            integrate_position(island.m_bodies, island.m_body_count, step.m_dt);

            write_back_status(island.m_bodies, island.m_body_count);
            // Post solve cleanup.

            for(int32 i = 0; i < island.get_body_count(); ++i) {
                // Allow static bodies to participate in other islands
                b3BodySim* body_sim = island.get_body(i);
                if(body_sim->body->m_type == b3BodyType::b3_static_body) {
                    body_sim->body->m_flags &= ~b3Body::e_island_flag;
                }
            }
            // clear all bodies and contacts count, so we can reuse the island for the next island.
            island.clear();
        }
    }

    void integrate_velocity(b3BodySim** body_sims, int32 body_count, real dt) {

        for(int32 i = 0; i < body_count; ++i) {

            b3BodySim* b = *(body_sims + i);

            b3Vec3r v = b->v;
            b3Vec3r w = b->w;

            if (b->inv_mass == 0) {
                continue;
            }

            v += dt * (m_gravity + b->inv_mass * (b->ext_force ));
            w += dt * b->inv_I * b->ext_torque;

            b->v = v;
            b->w = w;
        }
    }

    void integrate_position(b3BodySim** body_sims, int32 body_count, real dt) {

        for (int32 i = 0; i < body_count; ++i) {
            b3BodySim* b = *(body_sims + i);

            b3Vec3r p = b->p;
            b3Quatr q = b->q;

            const b3Vec3r& v = b->v;
            const b3Vec3r& w = b->w;

            p = p + v * dt;
            q = q + real(0.5) * dt * b3Quatr(0, w) * q;
            q.normalize();

            b->p = p;
            b->q = q;
        }
    }

    void write_back_status(b3BodySim** body_sims, int32 body_count) {
        for (int32 i = 0; i < body_count; ++i) {
            b3BodySim* b = *(body_sims + i);

            b3Body* body = b->body;

            body->m_p = b->p;
            body->m_q = b->q;
            body->m_v = b->v;
            body->m_w = b->w;
        }
    }
};


