// This BVH is implemented uppon Box2D.
// For more information, please access:
// https://github.com/erincatto/box2d

#ifndef BOX3D_B3_BVH_HPP
#define BOX3D_B3_BVH_HPP

#include "common/b3_types.hpp"

#include "collision/b3_aabb.hpp"

#define b3_NULL_NODE (-1)

#define b3_NULL_HEIGHT (-1)

#define b3_LEAF_HEIGHT (0)

namespace box3d {

    class b3Node;

    class b3DynamicTree;

}


class box3d::b3Node {

    friend class b3DynamicTree;

    /**
     * The AABB of the node.
     */
    b3AABB m_aabb;

    /**
     * Because the nodes are firstly allocated in a continuous memory,
     * we can use the index of the node to access its next node while in list
     * or its parent node while in tree.
     */
    union {
        int32 m_next;
        int32 m_parent;
    };

    /**
     * The index of left child of the node.
     */
    int32 m_child1;

    /**
     * The index of right child of the node.
     */
    int32 m_child2;

    /**
     * The height of the node in the dynamic tree.
     */
    int32 m_height;

public:

    /**
     * @brief Construct a new b3Node object
     */
    b3Node();

    /**
     * @brief Check whether the node is a leaf node.
     * @return
     */
    inline bool is_leaf() const {
        return m_child1 == b3_NULL_NODE;
    }

};

class box3d::b3DynamicTree {

    /**
     * The root node of the dynamic tree.
     */
    int32 m_root;

    /**
     * The nodes of the dynamic tree.
     */
    b3Node* m_nodes;

    /**
     * The number of nodes in the dynamic tree.
     */
    int32 m_node_count;

    /**
     * The maximum num of the nodes in the dynamic tree.
     */
    int32 m_node_capacity;

    /**
     * The free list of the nodes in the dynamic tree.
     */
    int32 m_free_list;

public:

    /**
     * @brief Construct a new b3DynamicTree object
     */
    b3DynamicTree();

    /**
     * @brief Destroy the b3DynamicTree object
     */
    ~b3DynamicTree();

    /**
     * @brief Create a proxy in the dynamic tree.
     * @param aabb: The AABB of the proxy.
     * @return The index of the proxy.
     */
    int32 create_bvh_proxy(const b3AABB& aabb);

    /**
     * @brief Destroy a proxy in the dynamic tree.
     * @param proxy_id: The index of the proxy.
     */
    void destroy_bvh_proxy(int32 proxy_id);

    bool move_proxy(int32 proxy_id, const b3AABB& aabb);

    const b3AABB& get_AABB(int32 proxy_id) const {

        b3_assert(0 <= proxy_id && proxy_id < m_node_capacity);

        return m_nodes[proxy_id].m_aabb;
    }

private:

    /**
     * @brief assign a leaf node to the dynamic tree.
     * @return The index of the leaf node.
     */
    int32 assign_node();

    /**
     * @brief Recycle a leaf node from the dynamic tree.
     * @param i_node: The index of the leaf node.
     */
    void recycle_node(int32 i_node);

    /**
     * @brief Insert a leaf node to the dynamic tree.
     * @param leaf: The index of the leaf node.
     */
    void insert_to_leaf(int32 leaf);

    /**
     * @brief Remove a leaf node from the dynamic tree.
     * @param leaf: The index of the leaf node.
     */
    void remove_leaf(int32 leaf);

    /**
     * @brief Balance the dynamic tree.
     * @param count
     */
    void expand_node_list(int32 count);

    /**
     * @brief Balance the dynamic tree.
     * @param i_A: The index of the node A.
     * @return The index of the root of this balanced subtree.
     */
    int32 balance(int32 i_A);

};


#endif //BOX3D_B3_BVH_HPP
