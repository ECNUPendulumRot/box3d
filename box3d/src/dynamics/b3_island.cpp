
#include "dynamics/b3_island.hpp"
#include "dynamics/b3_body.hpp"
#include "collision/b3_contact.hpp"

#include "common/b3_block_allocator.hpp"


b3Island::b3Island(b3BlockAllocator *block_allocator, 
				   int32 body_capacity, int32 contact_capacity) :
  m_body_capacity(body_capacity),
  m_contact_capacity(contact_capacity) {

  m_body_count = 0;
  m_contact_count = 0;

  void *mem = block_allocator->allocate(m_body_capacity * sizeof(b3Body *));
  m_bodies = new (mem) b3Body *;

  mem = block_allocator->allocate(m_contact_capacity * sizeof(b3Contact *));
  m_contacts = new (mem) b3Contact *;
}


void b3Island::add_body(b3Body *body) {
  b3_assert(m_body_count < m_body_capacity);

  body->set_island_index(m_body_count);
  m_bodies[m_body_count] = body;
  m_body_count++;
}


void b3Island::add_contact(b3Contact *contact) {
  b3_assert(m_contact_count < m_contact_capacity);

  m_contacts[m_contact_count] = contact;
  m_contact_count++;
}


b3Island::~b3Island() {
  b3_free(m_contacts);
}


void b3Island::solve(const double dt) {
  // Integrate velocities and apply damping is already finish

  // Init velocity constraints

  // Solve velocity constraints

  // Integrate position

  // Init position constraints

  // solve position constraints
}