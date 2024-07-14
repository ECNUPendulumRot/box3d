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


#ifndef BOX3D_B3_FIXTURE_HPP
#define BOX3D_B3_FIXTURE_HPP


#include "geometry/b3_shape.hpp"

#include "collision/b3_aabb.hpp"

/////////// Forward Declaration ///////////

class b3Fixture;

class b3BroadPhase;

//////////////////////////////////////////

/**
 * @brief Configure the properties and shape of fixtures used for
 * object simulation and collision detection in the physics engine.
 */
struct b3FixtureDef {

    /**
     * @brief Restitution coefficient, used to describe the energy
     * loss or rebound degree during collisions, defaulting to 0.3.
     */
    real m_restitution = 0.3;

    /**
     * @brief Friction coefficient describing the friction force
     * between colliding surfaces, defaulting to 0.0 (no friction).
     */
    real m_friction = 0.0;

    /**
     * @brief Density affecting the mass calculation of the fixture,
     * used to simulate the mass and inertia of objects, defaulting to 1.0.
     */
    real m_density = 1.0;

    /**
     * @brief Restitution threshold, used to control the elastic response
     * threshold in collisions, defaults to 0.0, with units in length units per meter.
     */
    real m_restitution_threshold = 0.0 * b3_length_units_per_meter;

    /**
     * @brief The shape of the fixture.
     * This shape must be allocated by the user.
     * It can be allocated on the stack because the shape will be cloned
     */
    b3Shape* m_shape = nullptr;

public:

    /**
     * @brief Get the restitution coefficient of the fixture.
     * @return return the restitution coefficient of the fixture.
     */
    inline real get_restitution() const {
        return m_restitution;
    }

    /**
     * @brief get the friction coefficient of the fixture.
     * @return Returns the friction coefficient of the fixture.
     */
    inline real get_friction() const {
        return m_friction;
    }

    /**
     * @brief get a pointer to the shape of the fixture.
     * @return Returns a pointer to the shape of the fixture.
     */
    inline b3Shape* get_shape() const {
        return m_shape;
    }

    /**
     * @brief get the density value of the fixture.
     * @return Returns the density value of the fixture.
     */
    inline real get_density() const {
        return m_density;
    }

};

/**
 * @brief Manages proxies for fixtures, containing the axis-aligned
 * bounding box (AABB) related to the fixture, a pointer to the
 * fixture itself, the proxy ID, and the proxy ID of the child shape.
 */
struct b3FixtureProxy {

    /**
     * @brief Defines a constant value -1 representing an invalid proxy ID.
     */
    enum {

        b3NullProxy = -1

    };

    /**
     * @brief The AABB of the fixture
     */
    b3AABB m_aabb;

    /**
     * @brief A pointer to the fixture object associated with this proxy.
     * Default value is nullptr.
     */
    b3Fixture* m_fixture = nullptr;

    /**
     * @brief ID of the proxy, used to uniquely identify the proxy.
     * Default value is -1.
     */
    int32 m_proxy_id = -1;

    /**
     * @brief the id of the proxy of the child shape
     */
    int32 m_child_id = -1;

    /**
     * @brief Get the fixture object (b3Fixture) associated with
     * the current proxy (b3FixtureProxy).
     * @return return the fixture object (b3Fixture) associated
     * with the current proxy (b3FixtureProxy).
     */
    inline b3Fixture* get_fixture() const {
        return m_fixture;
    }

};

/**
 * @brief A fixture used to represent an object in a physics engine,
 * containing physical properties, shape, and various information
 * and methods associated with the object.
 */
class b3Fixture {

    /**
     * @brief  b3Body class is declared as a friend of the b3Fixture class
     */
    friend class b3Body;

    /**
     * @brief b3ContactManager class is declared as a friend of the b3Fixture class
     */
    friend class b3ContactManager;

    /**
     * @brief Restitution coefficient, representing the energy loss or
     * degree of rebound during a collision.
     */
    real m_restitution = 0.0;

    /**
     * @brief Restitution threshold, used to control the elastic response threshold in collisions.
     */
    real m_restitution_threshold = 1.0 * b3_length_units_per_meter;

    /**
     * @brief Friction coefficient, describing the frictional force between collision surfaces.
     */
    real m_friction = 0.0;

    /**
     * @brief Density, affecting the mass calculation of the fixture, used
     * to simulate the mass and inertia of the object.
     */
    real m_density = 0.0;

    /**
     * @brief Pointer to the fixture's shape.
     */
    b3Shape* m_shape = nullptr;

    /**
     * @brief Pointer to the rigid body object associated with the fixture.
     */
    b3Body* m_body = nullptr;

    /**
     * @brief Pointer to the array of fixture proxies.
     */
    b3FixtureProxy* m_proxies = nullptr;

    /**
     * @brief Number of proxies
     */
    int32 m_proxy_count = 0;

    /**
     * @brief Pointer to the next fixture object
     */
    b3Fixture* m_next = nullptr;

public:

    /**
     * @brief Initializes the properties and shape of a fixture associated with a physics body.
     * @param block_allocator A pointer to a memory block allocator used to
     * manage memory allocations for the fixture and its proxies.
     * @param f_def  A constant reference to a b3FixtureDef object containing the definition of the fixture
     * @param body  A pointer to the physics body to which the fixture is attached
     */
    void create_fixture(b3BlockAllocator* block_allocator,  const b3FixtureDef& f_def, b3Body* body);

    /**
     * @brief creates proxies in the broad-phase collision detection system for each child shape of the fixture.
     * @param broad_phase A pointer to the broad-phase collision detection system where proxies are created
     * @param m_xf  A reference to a b3Transr object representing the transform of the fixture
     */
    void create_proxy(b3BroadPhase* broad_phase, b3Transr& m_xf);


    /**
     * @brief Computes the mass properties of the fixture's shape using the specified density.
     * @param mass_data Reference to a b3MassProperty object where the computed mass properties will be stored.
     */
    inline void get_mass_data(b3MassProperty& mass_data) const {
        m_shape->compute_mass_properties(mass_data, m_density);
    }

    /**
     * @brief Retrieves a pointer to the body associated with this fixture.
     * @return  Pointer to the b3Body object associated with the fixture.
     */
    b3Body* get_body() const {
        return m_body;
    }

    /**
     * @brief Retrieves a pointer to the next fixture attached to the same body.
     * @return Pointer to the next b3Fixture object in the linked list of fixtures.
     */
    b3Fixture* get_next() const {
        return m_next;
    }

    /**
     * @brief Retrieves a pointer to the shape associated with this fixture.
     * @return  Pointer to the b3Shape object representing the fixture's shape.
     */
    b3Shape* get_shape() const {
        return m_shape;
    }

    /**
     * @brief Sets the shape associated with this fixture to the provided shape.
     * @param shape  Pointer to the new b3Shape object to associate with the fixture.
     */
    void set_shape(b3Shape* shape) {
        m_shape = shape;
    }

    /**
     * @brief Retrieves the type of shape associated with this fixture.
     * @return Enum value (b3ShapeType) representing the type of shape.
     */
    inline b3ShapeType get_shape_type() const {
        return m_shape->get_type();
    }

    /**
     * @brief Retrieves a pointer to the fixture proxy at the specified index.
     * @param index  Index of the fixture proxy to retrieve.
     * @return Pointer to the b3FixtureProxy object at the specified index.
     */
    b3FixtureProxy* get_fixture_proxy(int index) {
        b3_assert(index >= 0 && index < m_proxy_count);
        return (m_proxies + index);
    }

    /**
     * @brief Retrieves the friction coefficient of the fixture.
     * @return return the friction coefficient
     */
    real get_friction() const {
        return m_friction;
    }

    /**
     * @brief Retrieves the restitution coefficient of the fixture.
     * @return return Retrieves the restitution coefficient
     */
    real get_restitution() const {
        return m_restitution;
    }

    /**
     * @brief Retrieves the restitution threshold of the fixture.
     * @return return the restitution threshold of the fixture.
     */
    real get_restitution_threshold() const {
        return m_restitution_threshold;
    }

private:

    /**
     * @brief updates the AABBs of the fixture proxies in the broad-phase collision detection system.
     * @param broad_phase  Pointer to the broad-phase collision detection system where proxies are managed.
     * @param transform1 The initial transformation (usually the previous state) of the body associated with this fixture.
     * @param transform2 The new transformation (current state) of the body associated with this fixture.
     */
    void synchronize(b3BroadPhase* broad_phase, const b3Transr& transform1, const b3Transr& transform2);

};


#endif //BOX3D_B3_FIXTURE_HPP
