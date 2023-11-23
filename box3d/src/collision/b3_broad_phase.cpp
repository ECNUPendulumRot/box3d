
#include "collision/b3_broad_phase.hpp"
#include "common/b3_allocator.hpp"

int32 box3d::b3BroadPhase::create_proxy(const box3d::b3AABB &aabb)
{
    int32 proxy_id = m_tree.create_bvh_proxy(aabb);
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
