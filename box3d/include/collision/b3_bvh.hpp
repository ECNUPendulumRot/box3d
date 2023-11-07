// This BVH is implemented uppon Box2D.
// For more information, please access:
// https://github.com/erincatto/box2d

#ifndef BOX3D_B3_BVH_HPP
#define BOX3D_B3_BVH_HPP

#include "common/b3_types.hpp"

#define b3_NULL_NODE (-1)

#define b3_NULL_HEIGHT (-1)

#define b3_LEAF_HEIGHT (0)

namespace box3d {

    class b3Node;

    class b3DynamicTree;

    class b3AABB;

}


class box3d::b3AABB {

    /**
     * The lower bound of the AABB.
     */
    b3Vector3f m_min;

    /**
     * The upper bound of the AABB.
     */
    b3Vector3f m_max;

public:

    b3AABB() = default;

    b3AABB(const b3Vector3f& lower_bound, const b3Vector3f& upper_bound);

    /**
     * @brief Get the center coordinate of the AABB.
     * @return The center coordinate of the AABB.
     */
    inline b3Vector3f center() const {
        return 0.5f * (m_max - m_min);
    }

    /**
     * @brief Get the surface area of the AABB.
     * @return The surface area of the AABB.
     */
    inline float get_surface_area(){
        float f1 =  (m_max.x() - m_min.x()) * (m_max.y() - m_min.y());
        float f2 =  (m_max.x() - m_min.x()) * (m_max.z() - m_min.z());
        float f3 =  (m_max.y() - m_min.y()) * (m_max.z() - m_min.z());
        return 2.0f * f1 * f2 * f3;
    }

    /**
     * @brief  Combine 2 AABBs into 1 AABB.
     */
    inline void combine(const b3AABB& A, const b3AABB& B) {
        m_min = b3_min_coeff(A.m_min, B.m_min);
        m_max = b3_max_coeff(A.m_max, B.m_max);
    }

    /**
     * @brief Check whether 2 AABBs are overlapped.
     * @param A: The first AABB.
     * @param B: The second AABB.
     * @details This method is introduced in @cite Real-Time Collision Detection, Chapter 4
     * @return True if 2 AABBs are overlapped, false otherwise.
     */
    static bool overlapped(const b3AABB& A, const b3AABB& B);

};


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
