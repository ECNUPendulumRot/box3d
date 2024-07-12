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


#ifndef BOX3D_CONTACT_HPP
#define BOX3D_CONTACT_HPP

#include "geometry/b3_shape.hpp"
#include "collision/b3_collision.hpp"

#include "common/b3_block_allocator.hpp"

/////////// Forward Declaration ///////////

class b3Body;
class b3Contact;
class b3Fixture;
class b3ContactListener;

//////////////////////////////////////////

/**
 * @brief Represents the collision link between two bodies,
 * containing information related to the collision.
 */
struct b3ContactEdge {
    /**
     * @brief the shape is contact with m_other, default value: nullptr.
     */
    b3Body* m_other = nullptr;
    /**
     * @brief A pointer to the b3Contact object representing
     * the current collision, default value: nullptr.
     */
    b3Contact* m_contact = nullptr;

    /**
     * @brief In all collision contacts of an object, this pointer points to
     * the previous collision contact of the current collision contact, default value: nullptr.
     */
    b3ContactEdge* m_prev = nullptr;

    /**
     * @brief this pointer points to the next collision contact of the current
     * collision contact. Default value: nullptr.
     */
    b3ContactEdge* m_next = nullptr;

};


/**
 * @brief Friction mixing law. The idea is to allow either fixture to drive the friction to zero.
 * @param friction1 The friction coefficient of the first fixture.
 * @param friction2 The friction coefficient of the second fixture.
 * @return Returns the square root of the product of the two friction coefficients,
 * representing the mixed friction value.
 */
inline real b3_mix_friction(real friction1, real friction2) {
    return b3_sqrt(friction1 * friction2);
}

/**
 * @brief Restitution mixing law. The idea is to allow for anything to bounce off an inelastic surface.
 * @param restitution1 The restitution coefficient of the first fixture.
 * @param restitution2 The restitution coefficient of the second fixture.
 * @return Returns the larger of the two restitution coefficients,
 * representing the mixed restitution coefficient.
 */
inline real b3_mix_restitution(real restitution1, real restitution2) {
    return restitution1 > restitution2 ? restitution1 : restitution2;
}

/**
 * @brief Restitution mixing law. This picks the lowest value.
 * @param threshold1 The restitution threshold of the first fixture
 * @param threshold2 The restitution threshold of the second fixture
 * @return Return the lesser of the two restitution thresholds,
 * representing the blended restitution threshold.
 */
inline float b3_mix_restitution_threshold(float threshold1, float threshold2)
{
    return threshold1 < threshold2 ? threshold1 : threshold2;
}

/**
 * @brief A function type used to create collision contacts between two fixtures.
 * @param fixture_A The first fixture involved in the collision
 * @param index_A Index of the first fixture in the tree
 * @param fixture_B The second fixture involved in the collision
 * @param index_B Index of the second fixture in the tree
 */
typedef b3Contact* b3ContactCreateFcn(b3Fixture* fixture_A, int32 index_A,
                                      b3Fixture* fixture_B, int32 index_B,
                                      b3BlockAllocator* block_allocator);

/**
 * @brief A function type used for destroying or cleaning up collision contacts
 * @param contact Pointer to the 'b3Contact' object to be destroyed
 * @param block_allocator  Used to free the memory associated with collision associations
 */
typedef void b3ContactDestroyFcn(b3Contact* contact, b3BlockAllocator* block_allocator);

/**
 * @brief Managing and handling the creation and destruction process of collision contacts.
 */
struct b3ContactRegister {

    /**
     * @brief A function pointer used to create a collision association
     */
    b3ContactCreateFcn* create_fcn;

    /**
     * @brief Pointer to a function used to destroy or clean collision associations
     */
    b3ContactDestroyFcn* destroy_fcn;

    /**
     * @brief Identifies whether the collision connection is the primary connection
     */
    bool primary;

};

/**
 * @brief Responsible for managing and describing collisions between two objects
 */
class b3Contact {

protected:

    /**
     * @brief The first fixture involved in the collision
     */
    b3Fixture* m_fixture_a;
    /**
     * @brief The second fixture involved in the collision
     */
    b3Fixture* m_fixture_b;

    /**
     * @brief The node associated with the first fixture in the collision
     */
    b3ContactEdge m_node_a;
    /**
     * @brief The node associated with the second fixture in the collision
     */
    b3ContactEdge m_node_b;

    /**
     * @brief Index of the first fixture
     */
    int32 m_index_a;
    /**
     * @brief Index of the second fixture
     */
    int32 m_index_b;

    ///////// for general linked list /////////
    /**
     * @brief Pointer to the previous node in the linked list.
     */
    b3Contact* m_prev;
    /**
     * @brief Pointer to the next node in the linked list.
     */
    b3Contact* m_next;

    /**
     * @brief Represents a collision manifold that stores contact points
     */
    b3Manifold m_manifold;

    /**
     * @brief Managing and handling the creation and destruction process of collision contacts.
     */
    static b3ContactRegister s_registers[b3ShapeType::e_type_count][b3ShapeType::e_type_count];

    /**
     * @brief Identifies whether the class has completed initialization
     */
    static bool s_initialized;


    ////////// Coefficients related to the material of the object ///////////

    /**
     * @brief Restitution coefficient of collision
     */
    real m_restitution;

    /**
     * @brief Friction coefficient of collision
     */
    real m_friction;

    /**
     * @brief Restitution threshold of collision
     */
    real m_restitution_threshold;

    /**
     * @brief flags marking collision states
     */
    uint32 m_flags = 0;

    /**
     * @brief Counter or variable related to time. The default value is 0.
     */
    int32 m_toi_count = 0;

    /**
     * @brief  represents a "Time of Impact" value in the class.
     */
    real m_toi;

public:

    /**
     * @brief Declares the b3World class as a friend metaclass of the current class b3Contact
     */
    friend class b3World;

    /**
     * @brief Constructor of the b3Contact class
     * @param f_A Pointer to the first fixture object
     * @param index_A The index of the first fixture in its parent object
     * @param f_B Pointer to the second fixture object
     * @param index_B The index of the second fixture in its parent object
     */
    b3Contact(b3Fixture* f_A, int32 index_A, b3Fixture* f_B, int32 index_B);

    /**
     * @brief Defines an enumeration that contains three bit flags
     */
    enum {
        /**
         * @brief this is used to generate island.
         */
        e_island_flag = 1,

        /**
         * @brief when aabb overlap, but two shapes maybe not intersecting.
         * so this flag is set when two shapes are intersecting or touching.
         */
        e_touching_flag = 1 << 1,

        /**
         * @brief Represents a sign used for Time of Impact (TOI)
         */
        e_toi_flag = 1 << 2
    };

    /**
     * @brief Set the flag to 1.
     * @param flag flag marking collision states
     */
    void set_flag(uint32 flag) {
        m_flags |= flag;
    }

    /**
     * @brief Set the flag to 0
     * @param flag flag marking collision states
     */
    void unset_flag(uint32 flag) {
        m_flags &= ~flag;
    }

    /**
     * @brief test of a flag is set
     * @param flag flag marking collision states
     * @return return test result of a flag
     */
    bool test_flag(uint32 flag) {
        if(m_flags & flag) {
            return true;
        }
        return false;
    }

    /**
     * @brief return pointers associated with the first fixtures involved in the collision contact.
     * @return Return pointers associated with the first fixtures involved in the collision contact.
     */
    inline b3Fixture* get_fixture_a() const {
        return m_fixture_a;
    }

    /**
     * @brief get pointers associated with the second fixtures involved in the collision contact.
     * @return Return pointers associated with the second fixtures involved in the collision contact.
     */
    inline b3Fixture* get_fixture_b() const {
        return m_fixture_b;
    }

    /**
     * @brief get pointers associated with the nodes of the first fixture involved in the collision contact.
     * @return Return pointers associated with the nodes of the first fixture involved in the collision contact.
     */
    b3ContactEdge* get_node_a() {
        return &m_node_a;
    }

    /**
     * @brief get pointers associated with the nodes of the second fixture involved in the collision contact.
     * @return Return pointers associated with the nodes of the second fixture involved in the collision contact.
     */
    b3ContactEdge* get_node_b() {
        return &m_node_b;
    }

    /**
     * @brief get a pointer to the previous collision contact in the bidirectional linked list.
     * @return Return a pointer to the previous collision contact in the bidirectional linked list.
     */
    inline b3Contact* prev() const {
        return m_prev;
    }

    /**
     * @brief get a pointer associated with the next collision contact in the bidirectional linked list.
     * @return return a pointer associated with the next collision contact in the bidirectional linked list.
     */
    inline b3Contact* next() const {
        return m_next;
    }

    /**
     * @brief Sets the next collision link for the current collision link
     * @param next Pointer to the next collision connection
     */
    inline void set_next(b3Contact* next) {
        m_next = next;
    }

    /**
     * @brief Set the previous contact of the current collision contact.
     * @param prev Pointer to the previous collision connection
     */
    inline void set_prev(b3Contact* prev) {
        m_prev = prev;
    }

    /**
     * @brief Set the pointer associated with the first fixture in the collision contact.
     * @param fixture_a Pointer to the first fixture.
     */
    inline void set_fixture_a(b3Fixture* fixture_a) {
        m_fixture_a = fixture_a;
    }

    /**
     * @brief Set the pointer associated with the second fixture in the collision contact.
     * @param fixture_b Pointer to the second fixture.
     */
    inline void set_fixture_b(b3Fixture* fixture_b) {
        m_fixture_b = fixture_b;
    }

    /**
     * Get the child index associated with the first fixture in the collision contact.
     * @return return the child index associated with the first fixture in the collision contact.
     */
    inline int32 get_child_index_a() const {
        return m_index_a;
    }

    /**
     * @brief Get the child index associated with the second fixture in the collision contact.
     * @return return the child index associated with the second fixture in the collision contact.
     */
    inline int32 get_child_index_b() const {
        return m_index_b;
    }

    /**
     * @brief Get the contact information of the collision contact.
     * @return return the contact information of the collision contact.
     */
    inline b3Manifold* get_manifold() {
        return &m_manifold;
    }

    /**
     * @brief Get the restitution coefficient of the collision.
     * @return return the restitution coefficient of the collision.
     */
    real get_restitution() const {
        return m_restitution;
    }

    /**
     * @brief Get the restitution threshold of the collision.
     * @return return the restitution threshold of the collision.
     */
    real get_restitution_threshold() const {
        return m_restitution_threshold;
    }

    /**
     * @brief Get the friction coefficient of the collision.
     * @return return the friction coefficient of the collision.
     */
    real get_friction() const {
        return m_friction;
    }

    /**
     * @brief Retrieve detailed information in the world coordinate system
     * for the collision contact, including collision points and normals
     * @param world_manifold Pointer to a b3WorldManifold object, used to store
     * computed collision information in world coordinates.
     */
    void get_world_manifold(b3WorldManifold* world_manifold) const;

    /**
     * @brief generate manifold between two shapes
     * @param manifold Used to store the calculated collision information
     * @param xfA Transformation information of the first shape
     * @param xfB Transformation information of the second shape
     */
    virtual void evaluate(b3Manifold* manifold, const b3Transr& xfA, const b3Transr& xfB) = 0;

protected:

    /**
     * @brief Manage collision detection and collision handling in physical simulations.
     */
    friend class b3ContactManager;

    /**
     * @brief Sets up collision handling methods for different shape combinations
     * in physical simulations, ensuring the generation and management of appropriate
     * collision types during collision detection and resolution processes.
     */
    static void initialize_registers();

    /**
     * @brief Associate the given shape combination with their
     * corresponding collision handling functions.
     * @param create_fcn A pointer to the function that creates a collision contact.
     * @param destroy_fcn A pointer to the function that destroys a collision contact.
     * @param type_A The first shape of the type
     * @param type_B The second shape of the type
     */
    static void add_type(b3ContactCreateFcn* create_fcn, b3ContactDestroyFcn* destroy_fcn,
                         b3ShapeType type_A, b3ShapeType type_B);

    /**
     * @brief Create a collision contact between two fixtures.
     * @param fixture_A Pointer to the first fixture involved in the collision
     * @param index_A Index of the first fixture
     * @param fixture_B Pointer to the second fixture involved in the collision
     * @param index_B Index of the second fixture
     * @param block_allocator Memory allocator used for allocating memory
     * @return If a suitable contact is found, the newly created b3Contact object is returned.
     * If no suitable contact is found, nullptr is returned.
     */
    static b3Contact* create(b3Fixture* fixture_A, int32 index_A, b3Fixture* fixture_B, int32 index_B, b3BlockAllocator* block_allocator);

    /**
     * @brief Used to destroy the specified 'b3Contact' object
     * and release its occupied memory resources.
     * @param contact A pointer to the `b3Contact` object that needs to be destroyed.
     * @param block_allocator A pointer to the memory allocator, used for managing and releasing memory.
     */
    static void destroy(b3Contact* contact, b3BlockAllocator* block_allocator);

    /**
     * @brief Update the contact manifold and touching status.
     * @param listener The listener is a pointer to the b3ContactListener
     * object for possible preprocessing event notifications
     */
    void update(b3ContactListener* listener);

};


#endif // BOX3D_CONTACT_HPP