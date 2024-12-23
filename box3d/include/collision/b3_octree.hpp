
#ifndef B3_OCTREE_HPP
#define B3_OCTREE_HPP

#include "collision/b3_aabb.hpp"

#include <vector>

class b3FixtureProxy;

class b3OCNode {
public:

    b3Vec3r m_center;

    b3Vec3r m_extend;

    int32 m_max_count = 4;

    int32 m_depth;

    // m_loose_bound = m_aabb * 2
    // 插入对象的中心点只要在格子中，那么它必定在这个格子的松散包围盒之内。
    // 所以只需要判断对象的中心属于哪个格子，就将其放入哪个节点中。（对象位于多个格子）
    // b3AABB m_loose_bound;

    // m_parent = (id - 1) / 8
    // int32 m_parent;

    std::vector<b3FixtureProxy*> m_objects;

    // child = 8 * id + 1, 8 * id + 2, 8 * id + 3, 8 * id + 4, 8 * id + 5, 8 * id + 6, 8 * id + 7, 8 * id + 8
    // int32 m_child[2][2][2];

    b3OCNode(b3AABB* aabb, int depth);
    ~b3OCNode();
};

class b3OCTree {

    // the max depth of this tree
    int m_max_depth;

    // the max number of objects in a node
    int m_max_obj_count;

    b3AABB m_aabb;

    // the root node of the tree
    int32 m_root;

    b3OCNode* m_nodes;
};

#endif