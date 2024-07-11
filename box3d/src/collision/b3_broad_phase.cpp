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


#include "collision/b3_broad_phase.hpp"
#include "collision/b3_fixture.hpp"
#include "common/b3_block_allocator.hpp"

/**
 * @brief constructor of b3BroadPhase
 */
b3BroadPhase::b3BroadPhase() {
    m_proxy_count = 0;

    m_pair_count = 0;
    m_pair_capacity = 16;

    // Allocate memory space after the initialization of m_block_allocator

    m_move_count = 0;
    m_move_capacity = 16;

}

/**
 * @brief Set the memory allocator.
 * @param block_allocator block_allocator is a pointer to an object of type b3BlockAllocator.
 */
void b3BroadPhase::set_block_allocator(b3BlockAllocator *block_allocator) {
    m_block_allocator = block_allocator;
    m_tree.set_block_allocator(block_allocator);

    m_pair_buffer = (b3Pair *)m_block_allocator->allocate(m_pair_capacity * sizeof(b3Pair));
    m_move_buffer = (int32 *)m_block_allocator->allocate(m_move_capacity * sizeof(int32));
}

/**
 * @brief Create a proxy and add a AABB to the dynamic tree.
 * @param aabb An object of type b3AABB.
 * @param fixture_proxy A proxy representing a fixture object,
 * used in collision detection systems to identify and manage fixtures of objects.
 * @return Returns the ID of the newly created proxy.
 */
int32 b3BroadPhase::create_proxy(const b3AABB &aabb, b3FixtureProxy *fixture_proxy) {
    int32 proxy_id = m_tree.create_bvh_proxy(aabb, fixture_proxy);

    ++m_proxy_count;

    // When created, all proxies are considered as moving.
    // TODO: check the buffer_move method
    buffer_move(proxy_id);

    return proxy_id;
}

/**
 * @brief Add the ID of the proxy that needs to be moved to the move buffer.
 * @param proxy_id the proxy id of AABB
 */
void b3BroadPhase::buffer_move(int32 proxy_id) {
    if (m_move_count == m_move_capacity) {
	    int32 *old_buffer = m_move_buffer;

	    m_move_capacity *= 2;

	    m_move_buffer = (int32 *)m_block_allocator->allocate(m_move_capacity * sizeof(int32));

	    memcpy(m_move_buffer, old_buffer, m_move_count * sizeof(int32));
	    m_block_allocator->free(old_buffer, m_move_count * sizeof(int32));
    }

    m_move_buffer[m_move_count] = proxy_id;
    ++m_move_count;
}

/**
 * @brief Update the position information of the specified proxy and ensure that the
 * corresponding data structure and state reflect these changes.
 * @param proxy_id the id of the proxy
 * @param aabb one AABB
 */
void b3BroadPhase::move_proxy(int32 proxy_id, const b3AABB &aabb) {
    bool buffer = m_tree.move_proxy(proxy_id, aabb);
    if (buffer) {
	    buffer_move(proxy_id);
    }
}

/**
 * @brief This is called by b3DynamicTree::query
 * when we are gathering pair
 * @param proxy_id the id of the proxy
*/
// bool 
void b3BroadPhase::query_callback(int32 proxy_id) {
    // self intersection
    if (proxy_id == m_query_proxy_id) {
	    return;
    }

    // box2d check two bodies are moving?
    // if all of them are moving, then cancel add this pair
    const bool moved = m_tree.was_moved(proxy_id);
    if (moved && proxy_id > m_query_proxy_id) {
  	    // avoid duplicate pairs
  	    return;
    }

    // extend the pair buffer as needed
    if (m_pair_count == m_pair_capacity) {

  	    b3Pair *old_buffer = m_pair_buffer;
  	    m_pair_capacity *= 2;
  	    m_pair_buffer = (b3Pair *)m_block_allocator->allocate(m_pair_capacity * sizeof(b3Pair));
  	    memcpy(m_pair_buffer, old_buffer, m_pair_count * sizeof(b3Pair));

  	    m_block_allocator->free(old_buffer, m_pair_count * sizeof(b3Pair));
    }

    // add pair
    m_pair_buffer[m_pair_count].proxy_id_a = b3_min(proxy_id, m_query_proxy_id);
    m_pair_buffer[m_pair_count].proxy_id_b = b3_max(proxy_id, m_query_proxy_id);
    ++m_pair_count;
}

