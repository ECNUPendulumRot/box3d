
#ifndef BOX3D_B3_BROAD_PHASE_HPP
#define BOX3D_B3_BROAD_PHASE_HPP

#include "collision/b3_bvh.hpp"
#include "common/b3_types.hpp"


/////////// Forward Delaration ///////////

class b3FixtureProxy;

class b3BlockAllocator;

//////////////////////////////////////////


struct b3Pair {

    int32 proxy_id_a;
    int32 proxy_id_b;

};


class b3BroadPhase {

    enum {
        e_null_proxy = -1
    };

    b3DynamicTree m_tree;

    int32 m_proxy_count = 0;

    /// Moving-related members are for coping with moving objects

    int32* m_move_buffer;

    int32 m_move_capacity = 16;

    int32 m_move_count = 0;

    b3Pair* m_pair_buffer;
    int32 m_pair_capacity = 16;
    int32 m_pair_count = 0;

    int32 m_query_proxy_id;

    b3BlockAllocator* m_block_allocator = nullptr;

public:

    b3BroadPhase();

    int32 create_proxy(const b3AABB& aabb, b3FixtureProxy* fixture_proxy);

    const b3AABB& get_AABB(int32 proxy_id) const {
        return m_tree.get_AABB(proxy_id);
    }

    void move_proxy(int32 proxy_id, const b3AABB& aabb);

    template<typename T>
    void update_pairs(T* callback);

    /**
     * @brief This is called by b3DynamicTree::query
     *          when we are gathering pair
    */
    // bool query_callback(int32 proxy_id);
    void query_callback(int32 proxy_id);

    void set_block_allocator(b3BlockAllocator* block_allocator);

private:

    void buffer_move(int32 proxy_id);

};


// TODO: move this to cpp file
template<typename T>
void b3BroadPhase::update_pairs(T* callback) {

    // reset pair buffer
    m_pair_count = 0;

    // perform tree queries for all moving proxies
    for(int i = 0; i < m_move_count; ++i) {
        m_query_proxy_id = m_move_buffer[i];

        // The moving proxy is removed from the buffer
        if(m_query_proxy_id == e_null_proxy) {
            continue;
        }

        // Note that the aabb is the fat aabb
        const b3AABB& aabb = get_AABB(m_query_proxy_id);

        m_tree.query(this, aabb);
    }

    // send pairs to caller
    for(int i = 0; i < m_pair_count; ++i) {
        b3Pair* current_pair = m_pair_buffer + i;
        b3FixtureProxy* fixture_proxy_a = m_tree.get_fixture_proxy(current_pair->proxy_id_a);
        b3FixtureProxy* fixture_proxy_b = m_tree.get_fixture_proxy(current_pair->proxy_id_b);

        callback->add_pair(fixture_proxy_a, fixture_proxy_b);

    }

    // clear all move flags
    // the move flag will set to true after the proxy is moved
    for (int32 i = 0; i < m_move_count; ++i)
    {
        int32 proxy_id = m_move_buffer[i];
        if (proxy_id == e_null_proxy)
        {
            continue;
        }

        m_tree.clear_moved(proxy_id);
    }

    // reset move buffer
    m_move_count = 0;
}


#endif //BOX3D_B3_BROAD_PHASE_HPP
