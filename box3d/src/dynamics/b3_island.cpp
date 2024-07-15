// The MIT License

// Copyright (c) 2024
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#include "dynamics/b3_island.hpp"
#include "dynamics/b3_body.hpp"
#include "collision/b3_contact.hpp"

#include "common/b3_block_allocator.hpp"

/**
 * @brief the constructor of b3Island class
 * @param block_allocator Allocator used for memory management.
 * @param body_capacity Maximum number of bodies the island can manage.
 * @param contact_capacity Maximum number of contacts the island can manage.
 */
b3Island::b3Island(b3BlockAllocator* block_allocator, int32 body_capacity, int32 contact_capacity):
    m_body_capacity(body_capacity), m_contact_capacity(contact_capacity)
{
    m_body_count = 0;
    m_contact_count = 0;
    m_block_allocator = block_allocator;

    void *mem = block_allocator->allocate(m_body_capacity * sizeof(b3Body *));
    m_bodies = new (mem) b3Body *;

    mem = block_allocator->allocate(m_contact_capacity * sizeof(b3Contact *));
    m_contacts = new (mem) b3Contact*;
}

/**
 * @brief Adds a body to the island.
 * @param body Pointer to the body to be added.
 */
void b3Island::add_body(b3Body* body)
{
    b3_assert(m_body_count < m_body_capacity);

    body->set_island_index(m_body_count);
    m_bodies[m_body_count] = body;
    m_body_count++;
}

/**
 * @brief Adds a contact to the island.
 * @param contact Pointer to the contact to be added.
 */
void b3Island::add_contact(b3Contact* contact)
{
    b3_assert(m_contact_count < m_contact_capacity);

    m_contacts[m_contact_count] = contact;
    m_contact_count++;
}

/**
 * @brief the destructor of b3Island class
 */
b3Island::~b3Island()
{
    m_block_allocator->free(m_bodies, m_body_capacity * sizeof(b3Body*));
    m_block_allocator->free(m_contacts, m_contact_capacity * sizeof(b3Contact*));
}
