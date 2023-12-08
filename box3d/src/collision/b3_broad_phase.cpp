
#include "collision/b3_broad_phase.hpp"
#include "common/b3_allocator.hpp"
#include "collision/b3_fixture.hpp"

int32 box3d::b3BroadPhase::create_proxy(const box3d::b3AABB &aabb, b3FixtureProxy* fixture_proxy)
{
    int32 proxy_id = m_tree.create_bvh_proxy(aabb, fixture_proxy);
    ++m_proxy_count;

    // When created, all proxies are considered as moving.
    // TODO: check the buffer_move method
    // This series of function is not sure.
    buffer_move(proxy_id);

    return proxy_id;
}


void box3d::b3BroadPhase::buffer_move(int32 proxy_id)
{
    if (m_move_count == m_move_capacity) {

        int32* old_buffer = m_move_buffer;

        m_move_capacity *= 2;

        m_move_buffer = (int32*)b3_alloc(m_move_capacity * sizeof(int32));

        memcpy(m_move_buffer, old_buffer, m_move_count * sizeof(int32));
        b3_free(old_buffer);
    }

    m_move_buffer[m_move_count] = proxy_id;
    ++m_move_count;
}


void box3d::b3BroadPhase::move_proxy(int32 proxy_id, const box3d::b3AABB &aabb)
{
    bool buffer = m_tree.move_proxy(proxy_id, aabb);
    if (buffer) {
        buffer_move(proxy_id);
    }
}

template<typename T>
void box3d::b3BroadPhase::update_pairs(T* callback) {
    
    m_pair_count = 0;

    for(int i = 0; i < m_move_count; ++i) {
        m_query_proxy_id = m_move_buffer[i];
        if(m_query_proxy_id == b3_NULL_NODE) {
            continue;
        }

        const b3AABB& aabb = get_AABB(m_query_proxy_id);

        m_tree.query(this, aabb);
    }

    for(int i = 0; i < m_pair_count; ++i) {
        b3Pair* current_pair = m_pair_buffer + i;
        b3FixtureProxy* fixture_proxy_a = m_tree.get_fixture_proxy(current_pair->proxy_id_a);
        b3FixtureProxy* fixture_proxy_b = m_tree.get_fixture_proxy(current_pair->proxy_id_b);

        callback->add_pair(fixture_proxy_a, fixture_proxy_b);

    }

    m_move_count = 0;
}

// bool 
void box3d::b3BroadPhase::query_callback(int32 proxy_id) {
    if(proxy_id == m_query_proxy_id) {
        return;
    } 

    // box2d check two bodies are moving? 
    // if all of them are moving, then cancel add this pair

    if(m_pair_count == m_pair_capacity) {
        b3Pair* old_buffer = m_pair_buffer;

        m_pair_capacity *= 2;

        m_pair_buffer = (b3Pair*)b3_alloc(m_pair_capacity * sizeof(b3Pair));

        memcpy(m_pair_buffer, old_buffer, m_pair_count * sizeof(b3Pair));
        b3_free(old_buffer);
    }

    // add pair
    m_pair_buffer[m_pair_count].proxy_id_a = b3_min(proxy_id, m_query_proxy_id);
    m_pair_buffer[m_pair_count].proxy_id_b = b3_max(proxy_id, m_query_proxy_id);
    ++m_pair_count;
}