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


#ifndef BOX3D_B3_BROAD_PHASE_HPP
#define BOX3D_B3_BROAD_PHASE_HPP

#include "collision/b3_bvh.hpp"
#include "common/b3_types.hpp"


/////////// Forward Declaration ///////////

class b3FixtureProxy;

class b3BlockAllocator;

//////////////////////////////////////////

/**
 * @brief Used to represent a pair of proxies. Each proxy has a unique ID,
 * with 'proxy_id_a' and 'proxy_id_b' representing the IDs of the two proxies, respectively.
 */
struct b3Pair {

    int32 proxy_id_a;
    int32 proxy_id_b;

};

/**
 * @brief Used to manage and handle operations related to the broad phase of collision detection.
 */
class b3BroadPhase {

    /**
     * @brief The invalid proxy ID is -1.
     */
    enum {
        e_null_proxy = -1
    };

    /**
     * @brief Dynamic tree structure used to store and manage the proxies' AABBs.
     */
    b3DynamicTree m_tree;

    /**
     * @brief The number of proxies, default is 0.
     */
    int32 m_proxy_count = 0;

    /// Moving-related members are for coping with moving objects\

    /**
     * @brief A buffer for proxies that need to be updated or moved.
     */
    int32* m_move_buffer;

    /**
     * @brief The initial capacity of 'm_move_buffer'.
     */
    int32 m_move_capacity = 16;

    /**
     * @brief The current number of AABBs that need to be moved, with a default value of 0.
     */
    int32 m_move_count = 0;

    /**
     * @brief The buffer that stores pairing information.
     */
    b3Pair* m_pair_buffer;

    /**
     * @brief The initial capacity of 'm_pair_buffer', which is 16 by default.
     */
    int32 m_pair_capacity = 16;

    /**
     * @brief The current number of stored pairs.
     */
    int32 m_pair_count = 0;

    /**
     * @brief The currently queried proxy ID.
     */
    int32 m_query_proxy_id;

    /**
     * @brief A pointer to b3BlockAllocator used for memory allocation.
     */
    b3BlockAllocator* m_block_allocator = nullptr;

public:

    /**
     * @brief constructor of b3BroadPhase
     */
    b3BroadPhase();

    ///// TODO: test dynamic tree ////////////
    /**
     * @brief Gets a pointer object to the dynamic tree of AABBs
     * @return Returns a pointer to the dynamic tree object of AABB
     */
    b3DynamicTree* get_dynamic_tree() {
        return &m_tree;
    }

    /**
     * @brief Create a proxy and add a AABB to the dynamic tree.
     * @param aabb An object of type b3AABB.
     * @param fixture_proxy A proxy representing a fixture object,
     * used in collision detection systems to identify and manage fixtures of objects.
     * @return Returns the ID of the newly created proxy.
     */
    int32 create_proxy(const b3AABB& aabb, b3FixtureProxy* fixture_proxy);

    /**
     * @brief Retrieve the specified AABB
     * @param proxy_id the id of the proxy
     * @return Return the AABB of the specified proxy based on its ID
     */
    const b3AABB& get_AABB(int32 proxy_id) const {
        return m_tree.get_AABB(proxy_id);
    }

    /**
     * @brief Update the position information of the specified proxy and ensure that the
     * corresponding data structure and state reflect these changes.
     * @param proxy_id the id of the proxy
     * @param aabb one AABB
     */
    void move_proxy(int32 proxy_id, const b3AABB& aabb);

    /**
     * @brief Update the proxy pairing information and call a callback function to handle the new pairs.
     * @tparam T T is a template parameter.
     * @param callback callback is a pointer to an object of type T, which is used to handle new proxy pairs.
     */
    template<typename T>
    void update_pairs(T* callback);

    /**
     * @brief This is called by b3DynamicTree::query
     * when we are gathering pair
     * @param proxy_id the id of the proxy
    */
    // bool query_callback(int32 proxy_id);
    void query_callback(int32 proxy_id);

    /**
     * @brief Set the memory allocator.
     * @param block_allocator block_allocator is a pointer to an object of type b3BlockAllocator.
     */
    void set_block_allocator(b3BlockAllocator* block_allocator);

    /**
     * @brief Test whether the AABBs of two proxies overlap.
     * @param proxy_id_a the first proxy id of AABBs
     * @param proxy_id_b the second proxy if of AABBs
     * @return True if 2 AABBs are overlapped, false otherwise.
     */
    inline bool test_overlap(int32 proxy_id_a, int32 proxy_id_b) const {
        const b3AABB& aabb_a = m_tree.get_AABB(proxy_id_a);
        const b3AABB& aabb_b = m_tree.get_AABB(proxy_id_b);
        return b3AABB::overlapped(aabb_a, aabb_b);
    }

private:

    /**
     * @brief Add the ID of the proxy that needs to be moved to the move buffer.
     * @param proxy_id the proxy id of AABB
     */
    void buffer_move(int32 proxy_id);

};


// TODO: move this to cpp file
/**
 * @brief Update the proxy pairing information and call a callback function to handle the new pairs.
 * @tparam T T is a template parameter.
 * @param callback callback is a pointer to an object of type T, which is used to handle new proxy pairs.
 */
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
