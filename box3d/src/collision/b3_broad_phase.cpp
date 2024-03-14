
#include "collision/b3_broad_phase.hpp"
#include "collision/b3_fixture.hpp"
#include "common/b3_block_allocator.hpp"

b3BroadPhase::b3BroadPhase()
{
    m_proxy_count = 0;

    m_pair_count = 0;
    m_pair_capacity = 16;

    // Allocate memory space after the initialization of m_block_allocator

    m_move_count = 0;
    m_move_capacity = 16;

}


void b3BroadPhase::set_block_allocator(b3BlockAllocator* block_allocator) {
    m_block_allocator = block_allocator;
    m_tree.set_block_allocator(block_allocator);

    m_pair_buffer = (b3Pair*)m_block_allocator->allocate(m_pair_capacity * sizeof(b3Pair));
    m_move_buffer = (int32*)m_block_allocator->allocate(m_move_capacity * sizeof(int32));
}


int32 b3BroadPhase::create_proxy(const b3AABB &aabb, b3FixtureProxy* fixture_proxy)
{
    int32 proxy_id = m_tree.create_bvh_proxy(aabb, fixture_proxy);

    ++m_proxy_count;

    // When created, all proxies are considered as moving.
    // TODO: check the buffer_move method
    buffer_move(proxy_id);

    return proxy_id;
}


void b3BroadPhase::buffer_move(int32 proxy_id)
{
    if (m_move_count == m_move_capacity) {

        int32* old_buffer = m_move_buffer;

        m_move_capacity *= 2;

        m_move_buffer = (int32*)m_block_allocator->allocate(m_move_capacity * sizeof(int32));

        memcpy(m_move_buffer, old_buffer, m_move_count * sizeof(int32));
        m_block_allocator->free(old_buffer, m_move_count * sizeof(int32));
    }

    m_move_buffer[m_move_count] = proxy_id;
    ++m_move_count;
}


void b3BroadPhase::move_proxy(int32 proxy_id, const b3AABB &aabb)
{
    bool buffer = m_tree.move_proxy(proxy_id, aabb);
    if (buffer) {
        buffer_move(proxy_id);
    }
}


// bool 
void b3BroadPhase::query_callback(int32 proxy_id)
{
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

        b3Pair* old_buffer = m_pair_buffer;
        m_pair_capacity *= 2;
        m_pair_buffer = (b3Pair*)m_block_allocator->allocate(m_pair_capacity * sizeof(b3Pair));
        memcpy(m_pair_buffer, old_buffer, m_pair_count * sizeof(b3Pair));

        m_block_allocator->free(old_buffer, m_pair_count * sizeof(b3Pair));
    }

    // add pair
    m_pair_buffer[m_pair_count].proxy_id_a = b3_min(proxy_id, m_query_proxy_id);
    m_pair_buffer[m_pair_count].proxy_id_b = b3_max(proxy_id, m_query_proxy_id);
    ++m_pair_count;
}

