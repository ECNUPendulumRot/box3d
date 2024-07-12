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


#ifndef B3_CONTACT_MANAGER
#define B3_CONTACT_MANAGER


#include "common/b3_types.hpp"
#include "collision/b3_broad_phase.hpp"
#include "common/b3_world_callback.hpp"


/////////// Forward Declaration ///////////

class b3FixtureProxy;

class b3Contact;

class b3BlockAllocator;

//////////////////////////////////////////

/**
 * @brief Manage collision detection and collision handling in physical simulations.
 */
class b3ContactManager {

    /**
     * @brief b3World class is a friend class of b3ContactManager.
     */
    friend class b3World;

    /**
     * @brief Broad-phase collision detection stage object
     */
    b3BroadPhase m_broad_phase;

    /**
     * @brief Pointer to the current contact list, default value is nullptr
     */
    b3Contact* m_contact_list = nullptr;

    /**
     * @brief Current number of contacts, default value is 0
     */
    int32 m_contact_count = 0;

    /**
     * @brief Pointer to the memory allocator, default value is nullptr
     */
    b3BlockAllocator* m_block_allocator = nullptr;

    /**
     * @brief Pointer to the collision listener, default value is nullptr
     */
    b3ContactListener* m_contact_listener = nullptr;

public:


    /**
     * @brief Find new collision contacts. When new fixtures are added
     * to the physical simulation, new collision contacts need to be found.
     */
    void find_new_contact();

    // Broad-phase callback, add a contact
    /**
     * @brief Add a collision contact between a pair of fixture proxies
     * (fixture_proxy_a and fixture_proxy_b) in the physical simulation.
     * If the two fixture proxies belong to different bodies and no
     * contact exists yet, a new contact is created and added to the
     * contact manager and the corresponding bodies.
     * @param fixture_proxy_a Pointer to the first fixture proxy
     * @param fixture_proxy_b Pointer to the second fixture proxy
     */
    void add_pair(b3FixtureProxy* fixture_proxy_a, b3FixtureProxy* fixture_proxy_b);

    /**
     * @brief destroy a contact and remove it from the contact list
     * @param contact A pointer to the b3Contact object to be destroyed
     */
    void destroy(b3Contact* contact);

    /**
     * @brief determine if the aabbs of two fixtures are overlapping,
     * if overlapping, will generate manifold.
     * if not overlapping, will destroy the contact.
     */
    void collide();

    /**
     * @brief Get the object for broad-phase collision detection stage
     * @return return the object for broad-phase collision detection stage
     */
    b3BroadPhase* get_broad_phase() {
        return &m_broad_phase;
    }

    /**
     * @brief get the current number of contacts
     * @return Return the current number of contacts
     */
    int32 get_contact_count() const {
        return m_contact_count;
    }

    /**
     * @brief get a pointer to the current contact list
     * @return Return a pointer to the current contact list
     */
    b3Contact* get_contact_list() const {
        return m_contact_list;
    }

    /**
     * @brief Set the block allocator for managing and allocating memory
     * @param block_allocator Pointer to the block allocator
     */
    void set_block_allocator(b3BlockAllocator* block_allocator) {
        m_block_allocator = block_allocator;
        m_broad_phase.set_block_allocator(block_allocator);
    }
};


#endif