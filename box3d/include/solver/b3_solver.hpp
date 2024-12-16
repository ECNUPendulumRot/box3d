
#pragma once

#include "common/b3_time_step.hpp"
#include "dynamics/b3_island.hpp"
#include "dynamics/b3_world.hpp"

struct b3Solver {

    b3World* m_world;
    b3BlockAllocator* m_block_allocator;


    enum {
        e_null_solver          = 0x001,
        e_substep_split_island = 0x002,
        e_jacobi_solver        = 0x003,
    };


    b3Solver(b3World* world, b3BlockAllocator* block_allocator) {
        m_world = world;
        m_block_allocator = block_allocator;
    }

    static b3Solver* create(b3World* world, b3BlockAllocator* block_allocator) {
        void* mem = block_allocator->allocate(sizeof(b3Solver));
        return new(mem) b3Solver(world, block_allocator);
    }

    static b3Solver* get_solver(uint32 solver_type, b3World* world, b3BlockAllocator* block_allocator);

    virtual void destroy() {
        this->~b3Solver();
        m_block_allocator->free(this, sizeof(b3Solver));
    }

    virtual void solve(b3TimeStep& step, b3BodySim* body_sims) {};

};

