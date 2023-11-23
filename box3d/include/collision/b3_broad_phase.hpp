
#ifndef BOX3D_B3_BROAD_PHASE_HPP
#define BOX3D_B3_BROAD_PHASE_HPP


#include "collision/b3_bvh.hpp"
#include "common/b3_types.hpp"

namespace box3d {

    class b3BroadPhase;

}


class box3d::b3BroadPhase {

    b3DynamicTree m_tree;

    int32 m_proxy_count = 0;

    /// Moving-related members are for coping with moving objects

    int32* m_move_buffer;

    int32 m_move_capacity = 16;

    int32 m_move_count = 0;

public:

    int32 create_proxy(const b3AABB& aabb);

    const b3AABB& get_AABB(int32 proxy_id) const {
        return m_tree.get_AABB(proxy_id);
    }

    void move_proxy(int32 proxy_id, const b3AABB& aabb);


private:

    void buffer_move(int32 proxy_id);


};

#endif //BOX3D_B3_BROAD_PHASE_HPP
