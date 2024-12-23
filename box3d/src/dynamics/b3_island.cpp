
#include "dynamics/b3_island.hpp"
#include "dynamics/b3_body.hpp"
#include "collision/b3_contact.hpp"
#include "dynamics/constraint/b3_constraint_base.hpp"

#include "common/b3_block_allocator.hpp"


b3Island::b3Island(b3BlockAllocator* block_allocator, int32 body_capacity, int32 contact_capacity, int32 constraint_capacity):
    m_body_capacity(body_capacity), m_contact_capacity(contact_capacity), m_constraint_capacity(constraint_capacity)
{
    m_body_count = 0;
    m_contact_count = 0;
    m_constraint_count = 0;
    m_block_allocator = block_allocator;

    void *mem = block_allocator->allocate(m_body_capacity * sizeof(b3Body *));
    m_bodies = new (mem) b3Body *;

    mem = block_allocator->allocate(m_contact_capacity * sizeof(b3Contact *));
    m_contacts = new (mem) b3Contact *;

    mem = block_allocator->allocate(m_constraint_capacity * sizeof(b3ConstraintBase*));
    m_constraints = new (mem) b3ConstraintBase *;
}


void b3Island::add_body(b3Body* body)
{
    b3_assert(m_body_count < m_body_capacity);

    body->set_island_index(m_body_count);
    m_bodies[m_body_count] = body;
    m_body_count++;
}


void b3Island::add_contact(b3Contact* contact)
{
    b3_assert(m_contact_count < m_contact_capacity);

    m_contacts[m_contact_count] = contact;
    m_contact_count++;
}

void b3Island::add_constraint(b3ConstraintBase* constraint) {
    b3_assert(m_constraint_count < m_constraint_capacity);

    m_constraints[m_constraint_count] = constraint;
    m_constraint_count++;
}

b3Island::~b3Island()
{
    m_block_allocator->free(m_bodies, m_body_capacity * sizeof(b3Body*));
    m_block_allocator->free(m_contacts, m_contact_capacity * sizeof(b3Contact*));
    m_block_allocator->free(m_constraints, m_constraint_capacity * sizeof(b3ConstraintBase*));
}
