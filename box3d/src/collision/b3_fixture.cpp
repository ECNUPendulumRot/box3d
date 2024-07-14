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


#include "collision/b3_fixture.hpp"

#include "collision/b3_broad_phase.hpp"
#include "dynamics/b3_body.hpp"

#include "common/b3_block_allocator.hpp"

/**
 * @brief Initializes the properties and shape of a fixture associated with a physics body.
 * @param block_allocator A pointer to a memory block allocator used to
 * manage memory allocations for the fixture and its proxies.
 * @param f_def  A constant reference to a b3FixtureDef object containing the definition of the fixture
 * @param body  A pointer to the physics body to which the fixture is attached
 */
void b3Fixture::create_fixture(
  b3BlockAllocator *block_allocator,
  const b3FixtureDef &f_def, b3Body *body)
{
    m_restitution = f_def.get_restitution();
    m_friction = f_def.get_friction();
    m_density = f_def.get_density();
    m_restitution_threshold = f_def.m_restitution_threshold;
    m_body = body;

    f_def.get_shape()->set_block_allocator(block_allocator);
    m_shape = f_def.get_shape()->clone();
    m_shape->set_relative_body(body);

    int32 child_count = m_shape->get_child_count();

    // m_proxies has as many proxies as number of children shapes
    m_proxies = (b3FixtureProxy *)block_allocator->allocate(child_count * sizeof(b3FixtureProxy));

    for (int32 i = 0; i < child_count; ++i) {
        m_proxies[i].m_fixture = nullptr;
        m_proxies[i].m_proxy_id = b3FixtureProxy::b3NullProxy;
    }

    m_proxy_count = 0;
}

/**
 * @brief creates proxies in the broad-phase collision detection system for each child shape of the fixture.
 * @param broad_phase A pointer to the broad-phase collision detection system where proxies are created
 * @param m_xf  A reference to a b3Transr object representing the transform of the fixture
 */
void b3Fixture::create_proxy(b3BroadPhase *broad_phase, b3Transr &m_xf)
{
    b3_assert(m_proxy_count == 0);
    m_proxy_count = m_shape->get_child_count();

    // create proxy for each child shape
    for (int32 i = 0; i < m_proxy_count; ++i) {
        b3FixtureProxy *proxy = m_proxies + i;
        m_shape->get_bound_aabb(&proxy->m_aabb, m_xf, i);
        proxy->m_proxy_id = broad_phase->create_proxy(proxy->m_aabb, proxy);
        proxy->m_fixture = this;
        proxy->m_child_id = i;
    }
}

/**
 * @brief updates the AABBs of the fixture proxies in the broad-phase collision detection system.
 * @param broad_phase  Pointer to the broad-phase collision detection system where proxies are managed.
 * @param transform1 The initial transformation (usually the previous state) of the body associated with this fixture.
 * @param transform2 The new transformation (current state) of the body associated with this fixture.
 */
void b3Fixture::synchronize(b3BroadPhase *broad_phase, const b3Transr &transform1, const b3Transr &transform2)
{
    if(m_proxy_count == 0) {
        return;
    }

    // the body position is updated, so we need to update the proxy's AABBs
    for(int32 i = 0; i <  m_proxy_count; ++i) {
        b3FixtureProxy* proxy = m_proxies + i;

        // Compute an AABB that covers the swept shape (may miss some rotation effect).
        b3AABB aabb1, aabb2;
        m_shape->get_bound_aabb(&aabb1, transform1, proxy->m_child_id);
        m_shape->get_bound_aabb(&aabb2, transform2, proxy->m_child_id);

        proxy->m_aabb.combine(aabb1, aabb2);

  	    broad_phase->move_proxy(proxy->m_proxy_id, proxy->m_aabb);
    }
}
