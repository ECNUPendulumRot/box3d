#pragma once

#include "b3_solver.hpp"
#include "solver/b3_contact_solver_gGS_substep.hpp"
#include "collision/b3_contact.hpp"

struct b3SolvergGSSubstepSplitIsland: b3Solver {
    int32 m_body_count;
    b3ContactManager* m_contact_manager;
    b3Vec3r m_gravity;

    b3SolvergGSSubstepSplitIsland(b3World* world, b3BlockAllocator* block_allocator):b3Solver(world, block_allocator) {

        m_body_count = world->get_body_count();
        m_contact_manager = &world->m_contact_manager;
        m_gravity = world->m_gravity;
    }

    static b3Solver* create(b3World* world, b3BlockAllocator* block_allocator) {
        void* mem = block_allocator->allocate(sizeof(b3SolvergGSSubstepSplitIsland));
        return new(mem) b3SolvergGSSubstepSplitIsland(world, block_allocator);
    }

    void destroy() override {
        this->~b3SolvergGSSubstepSplitIsland();
        m_block_allocator->free(this, sizeof(b3SolvergGSSubstepSplitIsland));
    }

    void solve(b3TimeStep &step, b3BodySim* body_sims) override {
        int32 substep = 4;
        real hw = step.m_hw;
        real dt_sub = real(1.0)/(substep * hw);

        b3StaticIsland static_contact_island(m_block_allocator, m_body_count, m_contact_manager->m_static_contact_count);
        b3NormalIsland normal_contact_island(m_block_allocator, m_body_count, m_contact_manager->m_normal_contact_count);

        // build static island
        for (b3Contact *contact = m_contact_manager->m_static_contact_list; contact; contact = contact->next()) {
            if (!contact->test_flag(b3Contact::e_touching_flag)) {
                continue;
            }

            static_contact_island.add_contact(contact);
        }

        // build normal island
        for (b3Contact *contact = m_contact_manager->m_contact_list; contact; contact = contact->next()) {
            if (!contact->test_flag(b3Contact::e_touching_flag)) {
                continue;
            }
            normal_contact_island.add_contact(contact);
        }

        b3ContactSolvergGS static_solver;
        static_solver.m_block_allocator = m_block_allocator;
        static_solver.m_island = &static_contact_island;
        static_solver.m_timestep = &step;
        static_solver.prepare_contact_contraints();

        b3ContactSolvergGS dynamic_solver;
        dynamic_solver.m_block_allocator = m_block_allocator;
        dynamic_solver.m_island = &normal_contact_island;
        dynamic_solver.m_timestep = &step;
        dynamic_solver.prepare_contact_contraints();

        // solve velocity constraints
        for (int32 i = 0; i < substep; i++) {
            integrate_velocity(body_sims, dt_sub); // integrate velocity

            if (!static_solver.empty_solver()) {
                static_solver.solve_velocity_constraints();
            }

            if (normal_contact_island.m_contact_count > 0) {
                dynamic_solver.solve_velocity_constraints();
            }
        }

        integrate_position(body_sims, real(1.0) / hw);

        write_back_status(body_sims);
    }

    void integrate_position(b3BodySim* body_sims, real dt) {

        for (int32 i = 0; i < m_body_count; ++i) {
            b3BodySim* b = body_sims + i;

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

    void write_back_status(b3BodySim *body_sims) {
        for (int32 i = 0; i < m_body_count; ++i) {
            b3BodySim* b = body_sims + i;

            b3Body* body = b->body;

            body->m_p = b->p;
            body->m_q = b->q;
            body->m_v = b->v;
            body->m_w = b->w;

           // std::cout << std::fixed << std::setprecision(7);
           // std::cout << "velocity: " << b->v[0] <<  " " << b->v[1] << " " << b->v[2] << std::endl;
        }
    }

    void integrate_velocity(b3BodySim* body_sims, real dt) {

        for(int32 i = 0; i < m_body_count; ++i) {

            b3BodySim* b = body_sims + i;

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
};