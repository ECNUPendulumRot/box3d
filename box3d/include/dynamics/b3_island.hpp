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


#ifndef BOX3D_B3_ISLAND_HPP
#define BOX3D_B3_ISLAND_HPP


#include "common/b3_types.hpp"


/////////// Forward Declaration ///////////

class b3Contact;

class b3Body;

class b3BlockAllocator;

//////////////////////////////////////////

/**
 * @brief The b3Island class is designed to manage a group of b3Body and b3Contact
 * objects during a simulation step in a physics engine.
 */
class b3Island {

    /**
     * @brief Tracks the current number of bodies in the island.
     */
    int32 m_body_count;
    /**
     * @brief Tracks the current number of contacts in the island.
     */
	int32 m_contact_count;

    /**
     * @brief Sets the maximum number of bodies the island can manage.
     */
	int32 m_body_capacity;
    /**
     * @brief Sets the maximum number of contacts the island can manage.
     */
	int32 m_contact_capacity;

    /**
     * @brief Holds pointers to the bodies within the island.
     */
    b3Body** m_bodies;
    /**
     * @brief Holds pointers to the contacts within the island.
     */
    b3Contact** m_contacts;

    /**
     * @brief Manages memory allocation for the island.
     */
    b3BlockAllocator* m_block_allocator;

public:

    /**
     * @brief the constructor of b3Island class
     * @param block_allocator Allocator used for memory management.
     * @param body_capacity Maximum number of bodies the island can manage.
     * @param contact_capacity Maximum number of contacts the island can manage.
     */
    b3Island(b3BlockAllocator* block_allocator, int32 body_capacity, int32 contact_capacity);

    /**
     * @brief the destructor of b3Island class
     */
    ~b3Island();

    /**
     * @brief Adds a body to the island.
     * @param body Pointer to the body to be added.
     */
    void add_body(b3Body* body);

    /**
     * @brief Adds a contact to the island.
     * @param contact Pointer to the contact to be added.
     */
    void add_contact(b3Contact* contact);

    /**
     * @brief Resets the island by clearing the count of bodies and contacts.
     */
    void clear() {
        m_body_count = 0;
        m_contact_count = 0;
    }

    /**
     * @brief Retrieves the array of contacts.
     * @return returns the array of contacts.
     */
    b3Contact** get_contacts() const {
        return m_contacts;
    }

    /**
     * @brief Gets the number of contacts in the island.
     * @return returns the number of contacts in the island.
     */
    int get_contacts_count() const {
        return m_contact_count;
    }

    /**
     * @brief Gets the number of bodies in the island.
     * @return returns the number of bodies in the island.
     */
    int get_body_count() const {
        return m_body_count;
    }

    /**
     * @brief Retrieves the array of bodies.
     * @return returns the array of bodies.
     */
    b3Body** get_bodies() const {
        return m_bodies;
    }

    /**
     * @brief Retrieves a specific body by index.
     * @param index Index of the body to retrieve.
     * @return Pointer to the b3Body at the specified index.
     */
    b3Body* get_body(int32 index) {
        return m_bodies[index];
    }
};


#endif // BOX3D_B3_ISLAND_HPP