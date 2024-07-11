// The MIT License

// Copyright (c) 2024
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


// This BVH is implemented uppon Box2D.
// For more information, please access:
// https://github.com/erincatto/box2d

#ifndef BOX3D_B3_BVH_HPP
#define BOX3D_B3_BVH_HPP

#include "common/b3_types.hpp"

#include "collision/b3_aabb.hpp"

#include <stack>


/////////// Forward Declaration ///////////

class b3FixtureProxy;

class b3BlockAllocator;

//////////////////////////////////////////


#define b3_NULL_NODE (-1) ///< b3_NULL_NODE is a constant with a default value of -1, representing an invalid or empty node.

#define b3_NULL_HEIGHT (-1) ///< b3_NULL_HEIGHT is a constant with a value of -1, indicating an invalid or undefined tree height.

#define b3_LEAF_HEIGHT (0) ///< The constant b3_LEAF_HEIGHT has a value of 0 and represents the height of a leaf node.

/**
 * @brief The b3Node class defines nodes within a dynamic tree.
 */
class b3Node {

    /**
     * @brief The class b3DynamicTree is a friend class of b3Node.
     */
    friend class b3DynamicTree;

    /**
     * @brief The AABB of the node.
     */
    b3AABB m_aabb;

    /**
     * @brief The proxy of the fixture
    */
    b3FixtureProxy* fixture_proxy;
    /**
     * @brief Because the nodes are firstly allocated in a continuous memory,
     * we can use the index of the node to access its next node while in list
     * or its parent node while in tree.
     */
    union {
        int32 m_next;
        int32 m_parent;
    };

    /**
     * @brief The index of left child of the node.
     */
    int32 m_child1;

    /**
     * @brief The index of right child of the node.
     */
    int32 m_child2;

    /**
     * @brief The height of the node in the dynamic tree.
     */
    int32 m_height;

    /**
     * @brief Indicates whether the node has been moved.
     */
    bool m_moved;

public:

    /**
     * @brief Construct a new b3Node object
     */
    b3Node();

    /**
     * @brief Check whether the node is a leaf node.
     * @return true indicates that the current node is a leaf node,
     * false indicates that the current node is not a leaf node.
     */
    inline bool is_leaf() const {
        return m_child1 == b3_NULL_NODE;
    }

};

/**
 * @brief "b3DynamicTree" class manages the overall structure and operations of the dynamic tree.
 */
class b3DynamicTree {

    /**
     * @brief The root node of the dynamic tree.
     */
    int32 m_root;

    /**
     * @brief The nodes of the dynamic tree.
     */
    b3Node* m_nodes;

    /**
     * @brief The number of nodes in the dynamic tree.
     */
    int32 m_node_count;

    /**
     * @brief The maximum num of the nodes in the dynamic tree.
     */
    int32 m_node_capacity;

    /**
     * @brief The free list of the nodes in the dynamic tree.
     */
    int32 m_free_list;

    /**
     * @brief memory block allocator
     */
    b3BlockAllocator* m_block_allocator = nullptr;

public:

    /**
     * @brief Construct a new b3DynamicTree object
     */
    b3DynamicTree();

    /**
     * @brief Set the memory allocator
     * @param block_allocator block_allocator is a pointer to an object of type b3BlockAllocator.
     */
    void set_block_allocator(b3BlockAllocator* block_allocator);

    /**
     * @brief destroy the b3DynamicTree object
     */
    ~b3DynamicTree();

    /**
     * @brief create a proxy in the dynamic tree.
     * @param aabb: The AABB of the proxy.
     * @param user_data: general is proxy of the fixture
     * @return The index of the proxy.
     */
    int32 create_bvh_proxy(const b3AABB& aabb, b3FixtureProxy* user_data);

    /**
     * @brief destroy a proxy in the dynamic tree.
     * @param proxy_id: The index of the proxy.
     */
    void destroy_bvh_proxy(int32 proxy_id);

    /**
     * @brief Move the position of a specified node in the dynamic tree,
     * ensuring that the structure of the dynamic tree and the relationships
     * between nodes' positions are correctly maintained.
     * @param proxy_id the id of the proxy
     * @param aabb one AABB
     * @return Returns true if the node is successfully moved; otherwise, returns false.
     */
    bool move_proxy(int32 proxy_id, const b3AABB& aabb);

    /**
     * @brief Check if the specified node has been moved within the dynamic tree.
     * @param proxyId ID of the node to check.
     * @return If the node has been moved, it returns true; otherwise, it returns false.
     */
    inline bool was_moved(int32 proxyId) const {
        b3_assert(0 <= proxyId && proxyId < m_node_capacity);
        return m_nodes[proxyId].m_moved;
    }

    /**
     * @brief Reset the movement status of the specified node in the dynamic tree.
     * @param proxyId ID of the node to check.
     */
    inline void clear_moved(int32 proxyId)
    {
        b3_assert(0 <= proxyId && proxyId < m_node_capacity);
        m_nodes[proxyId].m_moved = false;
    }

    /**
     * @brief get the AABB of the specified node in the dynamic tree.
     * @param proxy_id ID of the proxy
     * @return Returns the AABB of the specified node in the dynamic tree.
     */
    inline const b3AABB& get_AABB(int32 proxy_id) const {

        b3_assert(0 <= proxy_id && proxy_id < m_node_capacity);

        return m_nodes[proxy_id].m_aabb;
    }

    /**
     * @brief get the fixture proxy of the specified node in the dynamic tree.
     * @param proxy_id ID of the proxy
     * @return Returns the fixture proxy of the specified node in the dynamic tree.
     */
    inline b3FixtureProxy* get_fixture_proxy(int32 proxy_id) const {
        b3_assert(0 <= proxy_id && proxy_id < m_node_capacity);
        return m_nodes[proxy_id].fixture_proxy;
    }

    /**
     * @brief iterator the tree to find overlap AABB
     * @param aabb AABB represents the AABB bounding box to be queried.
     * @param callback Callback: Used to process query results.
    */
    template<typename T>
    void query(T* callback, const b3AABB& aabb) const;


    /// TODO: this function is used to test dynamic tree, delete this function
    /**
     * @brief This function get the capacity of nodes currently allocated in the dynamic tree.
     * @return returns the capacity of nodes
     */
    int get_node_capacity() const {
        return m_node_capacity;
    }

    /**
     * @brief This function returns the current count of nodes that
     * are actively in use within the dynamic tree structure.
     * @return returns the current count of nodes
     */
    int get_node_count() const {
        return m_node_count;
    }

    /**
     * @brief get information about a specified node in the dynamic tree.
     * @param id Indicates the index id of a node
     * @param height Specifies the height of the returned node
     * @param aabb  The AABB to retrieve
     */
    void get_node_info(int32 id, int32& height, b3AABB& aabb) {
        height = m_nodes[id].m_height;
        aabb = m_nodes[id].m_aabb;
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
     * @brief Expand the node capacity of dynamic tree.
     * @param count: The count of new m_node_capacity
     */
    void expand_node_list(int32 count);

    /**
     * @brief Balance the dynamic tree.
     * @param i_A: The index of the node m_.
     * @return The index of the root of this balanced subtree.
     */
    int32 balance(int32 i_A);

};

/**
 * @brief iterator the tree to find overlap AABB
 * @param aabb AABB represents the AABB bounding box to be queried.
 * @param callback Callback: Used to process query results.
*/
template<typename T>
void b3DynamicTree::query(T* callback, const b3AABB& aabb) const {

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
