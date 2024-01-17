// This BVH is implemented uppon Box2D.
// For more information, please access:
// https://github.com/erincatto/box2d

#ifndef BOX3D_B3_BVH_HPP
#define BOX3D_B3_BVH_HPP

#include "common/b3_types.hpp"

#include "collision/b3_aabb.hpp"

#include <stack>


#define b3_NULL_NODE (-1)

#define b3_NULL_HEIGHT (-1)

#define b3_LEAF_HEIGHT (0)


namespace box3d {

    class b3Node;

    class b3DynamicTree;

    class b3FixtureProxy;

}


class box3d::b3Node {

    friend class b3DynamicTree;

    /**
     * The AABB of the node.
     */
    b3AABB m_aabb;

    /**
     * general
     * The proxy of the fxiture
    */
    b3FixtureProxy* fixture_proxy;
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

    bool m_moved;

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
     * @param user_data: general is proxy of the fixture
     * @return The index of the proxy.
     */
    int32 create_bvh_proxy(const b3AABB& aabb, b3FixtureProxy* user_data);

    /**
     * @brief Destroy a proxy in the dynamic tree.
     * @param proxy_id: The index of the proxy.
     */
    void destroy_bvh_proxy(int32 proxy_id);

    bool move_proxy(int32 proxy_id, const b3AABB& aabb);

    inline bool was_moved(int32 proxyId) const {
        b3_assert(0 <= proxyId && proxyId < m_node_capacity);
        return m_nodes[proxyId].m_moved;
    }

    inline void clear_moved(int32 proxyId)
    {
        b3_assert(0 <= proxyId && proxyId < m_node_capacity);
        m_nodes[proxyId].m_moved = false;
    }

    const b3AABB& get_AABB(int32 proxy_id) const {

        b3_assert(0 <= proxy_id && proxy_id < m_node_capacity);

        return m_nodes[proxy_id].m_aabb;
    }

    inline b3FixtureProxy* get_fixture_proxy(int32 proxy_id) const {
        b3_assert(0 <= proxy_id && proxy_id < m_node_capacity);
        return m_nodes[proxy_id].fixture_proxy;
    }

    /**
     * @brief iterator the tree to find overlap AABB
    */
    template<typename T>
    void query(T* callback, const b3AABB& aabb) const;

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


template<typename T>
void box3d::b3DynamicTree::query(T* callback, const b3AABB& aabb) const {

    std::stack<int32> stack;
    stack.push(m_root);

    while (stack.size() > 0) {
        int32 node_id = stack.top(); 
        stack.pop();
        if(node_id == b3_NULL_NODE) {
            continue;
        }

        const b3Node* node = m_nodes + node_id;

        if(b3AABB::overlapped(node->m_aabb, aabb)) {
            if(node->is_leaf()) {
                // TODO: check here
                callback->query_callback(node_id);
            } else {
                stack.push(node->m_child1);
                stack.push(node->m_child2);
            }
        }
    }
}


#endif //BOX3D_B3_BVH_HPP
