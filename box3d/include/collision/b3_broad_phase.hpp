
#ifndef BOX3D_B3_BROAD_PHASE_HPP
#define BOX3D_B3_BROAD_PHASE_HPP


#include "collision/b3_bvh.hpp"
#include "common/b3_types.hpp"

namespace box3d {

    class b3BroadPhase;

    class b3Pair;

    class b3FixtureProxy;
}

class box3d::b3Pair {

public:

    int32 proxy_id_a;
    int32 proxy_id_b;
};


class box3d::b3BroadPhase {

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

public:

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

private:

    void buffer_move(int32 proxy_id);


};

#endif //BOX3D_B3_BROAD_PHASE_HPP
