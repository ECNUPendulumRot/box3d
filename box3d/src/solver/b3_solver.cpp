
#include "solver/b3_solver.hpp"

#include "dynamics/b3_island.hpp"
#include "dynamics/b3_body.hpp"
#include "collision/b3_collision.hpp"
#include "collision/b3_contact.hpp"
#include "common/b3_time_step.hpp"
#include "common/b3_block_allocator.hpp"
#include "collision/b3_fixture.hpp"


void b3Solver::init(b3BlockAllocator *block_allocator, b3Island *island, b3TimeStep *step)
{
    m_method = step->m_integral_method;
    m_timestep = step;
    m_block_allocator = block_allocator;

    m_contact_count = island->get_contacts_count();
    m_contacts = island->get_contacts();

    m_body_count = island->get_body_count();
    b3_assert(m_body_count > 0);
    m_bodies = island->get_bodies();

    b3_assert(m_contact_count >= 0);

    void* memory = m_block_allocator->allocate(m_body_count * sizeof(Eigen::Vector<real, 12>));
    m_affine_qs = new (memory) Eigen::Vector<real, 12>;

    memory = m_block_allocator->allocate(m_body_count * sizeof(Eigen::Vector<real, 12>));
    m_affine_q_dots = new (memory) Eigen::Vector<real, 12>;

    memory = m_block_allocator->allocate(m_body_count * sizeof(Eigen::Vector<double, 12>));
    m_affine_q_preds = new (memory) Eigen::Vector<double, 12>;

    memory = m_block_allocator->allocate(m_body_count * sizeof(double));
    m_ks = new (memory) double;

    memory = m_block_allocator->allocate(m_body_count * sizeof(double));
    m_vs = new (memory) double;

    memory = m_block_allocator->allocate(m_body_count * sizeof(Eigen::Matrix<double, 12, 12>));
    m_Ms = new (memory) Eigen::Matrix<double, 12, 12>;


    if(m_contact_count > 0) {
        memory = m_block_allocator->allocate(m_contact_count * sizeof(b3AffineContactVelocityConstraint));
        m_avc = new (memory) b3AffineContactVelocityConstraint;
    } else {
        m_avc = nullptr;
    }

    const real& delta_t = m_timestep->m_dt;
    for(int32 i = 0; i < m_body_count; ++i) {
        b3Body* b = m_bodies[i];
        auto g = b->get_affine_gravity();

        m_ks[i] = b->get_stiffness();
        m_vs[i] = b->get_volume();
        m_affine_qs[i] = b->get_affine_q();
        m_affine_q_dots[i] = b->get_affine_q_dot();
        m_affine_q_preds[i] = (m_affine_qs[i] + delta_t * m_affine_q_dots[i] + delta_t * delta_t * g).cast<double>();
        m_Ms[i] = b->get_affine_mass().cast<double>();
    }
}


void b3Solver::write_states_back()
{
    for(int32 i = 0; i < m_body_count; ++i) {
        b3Body *b = m_bodies[i];
        b->set_affine_q(m_affine_qs[i]);
        b->set_affine_q_dot(m_affine_q_dots[i]);
        // TODO: if we need SynchronizeTransform() ?
    }
}


void b3Solver::clear() {
    m_timestep = nullptr;
    m_contacts = nullptr;

    m_block_allocator->free(m_avc, m_contact_count * sizeof(b3AffineContactVelocityConstraint));
    m_block_allocator->free(m_affine_qs, m_body_count * sizeof(Eigen::Vector<real, 12>));
    m_block_allocator->free(m_affine_q_dots, m_body_count * sizeof(Eigen::Vector<real, 12>));
    m_block_allocator->free(m_affine_q_preds, m_body_count * sizeof(Eigen::Vector<real, 12>));
    m_block_allocator->free(m_ks, m_body_count * sizeof(double));
    m_block_allocator->free(m_vs, m_body_count * sizeof(double));
    m_block_allocator->free(m_Ms, m_body_count * sizeof(Eigen::Matrix<double, 12, 12>));
    m_block_allocator = nullptr;
}


