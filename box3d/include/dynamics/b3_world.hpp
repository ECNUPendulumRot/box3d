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


#ifndef BOX3D_B3_WORLD_HPP
#define BOX3D_B3_WORLD_HPP

#include <filesystem>

#include "dynamics/b3_body.hpp"
#include "dynamics/b3_body_def.hpp"

#include "geometry/b3_shape.hpp"

#include "collision/b3_broad_phase.hpp"
#include "collision/b3_contact_manager.hpp"

#include "dynamics/b3_island.hpp"

#include "common/b3_block_allocator.hpp"

/////////// Forward Declaration ///////////

struct b3Color;
struct b3TimeStep;
class b3Draw;

//////////////////////////////////////////

/**
 * @brief The b3World class represents a physical simulation environment where bodies
 * interact under physics rules such as gravity, contact management, and constraints
 * solving.
 */
class b3World {

    /**
     * @brief This is the all bodies in the world.
     * doubly linked list
     */
    b3Body* m_body_list;

    /**
     * @brief This is the all shapes in the world.
     * doubly linked list
     */
    b3Shape* m_shape_list;

    /**
     * @brief Count of shapes currently in the world.
     */
    int32 m_shape_count;

    /**
     * @brief Count of bodies currently in the world.
     */
    int32 m_body_count;

    /**
     * @brief Vector representing the gravitational force acting on bodies.
     */
    b3Vec3r m_gravity = b3Vec3r(0, 0, 0);
    /**
     * @brief Frequency of physics steps (default is 60 Hz).
     */
    real m_hz = 60;
    /**
     * @brief Memory allocator for managing dynamic memory in the world.
     */
    b3BlockAllocator m_block_allocator;
    /**
     * @brief Manages contacts and collision detection.
     */
    b3ContactManager m_contact_manager;
    /**
     * @brief Flag indicating if new contacts were found.
     */
    bool m_new_contacts = false;
    /**
     * @brief Flag controlling whether bodies are allowed to sleep.
     */
    bool m_allow_sleep = false;
    /**
     * @brief Flag controlling continuous physics simulation.
     */
    bool m_continuous_physics = false;
    /**
     * @brief Flag indicating if the current physics step is complete.
     */
    bool m_step_complete = true;
    /**
     * @brief Pointer to a debug drawing interface for visualizing the world.
     */
    b3Draw* m_debug_draw;

public:

    /**
     * @brief Constructor to initialize a world with default settings.
     */
    b3World();

    /**
     * @brief Constructor to initialize a world with a specified gravity vector.
     * @param gravity Vector representing the gravitational force acting on bodies.
     */
    explicit b3World(const b3Vec3r& gravity);

    /**
     * @brief Destructor of b3World
     */
    ~b3World();

    /**
     * @brief Creates a new body in the world based on the provided body definition.
     * @param def Definition of the body to be created.
     * @return Pointer to the created body.
     */
    b3Body* create_body(const b3BodyDef& def);

    /**
     * @brief determine if the b3World object has no bodies currently present in its simulation.
     * @return ture if the world has no bodies,false if the world contains bodies
     */
    inline bool empty() const {
        return m_body_count == 0;
    }

    /**
     * @brief Advances the physics simulation by one time step.
     * @param dt Time step size.
     * @param velocity_iterations Iterations for velocity updates.
     * @param position_iterations Iterations for position updates.
     */
    void step(real dt, int32 velocity_iterations, int32 position_iterations);

    /**
     * @brief Sets whether bodies are allowed to enter sleep mode.
     * @param flag true to allow sleeping, false to disable sleeping.
     */
    void set_allow_sleeping(bool flag);

    /**
     * @brief Draws debug information about bodies and shapes in the world.
     */
    void debug_draw();

    /**
     * @brief retrieves the count of shapes currently present in the b3World object.
     * @return returns the count of shapes currently present in the b3World object.
     */
    inline int get_shape_count() const {
        return m_shape_count;
    }

    /**
     * @brief gets a pointer to the linked list of shapes (m_shape_list) in the world.
     * @return Returns a pointer to the linked list of shapes (m_shape_list) in the world.
     */
    b3Shape* get_shape_list() const {
        return m_shape_list;
    }

    /**
     * @brief Adds a shape to the world
     * @param shape Pointer to the shape to add.
     */
    void add_shape(b3Shape* shape);

    /**
     * @brief Sets the gravitational force vector of the b3World object.
     * @param gravity Takes a constant reference to a b3Vec3r object (gravity),
     * representing the new gravity vector.
     */
    void set_gravity(const b3Vec3r& gravity) {
        m_gravity = gravity;
    }

    /**
     * @brief Retrieves the current gravity vector (m_gravity) of the b3World.
     * @return returns the current gravity vector (m_gravity) of the b3World.
     */
    inline const b3Vec3r& gravity() {
        return m_gravity;
    }

    /**
     * @brief Retrieves the broad-phase collision detection manager.
     * @return returns the broad-phase collision detection manager.
     */
    b3BroadPhase* get_broad_phase() {
        return m_contact_manager.get_broad_phase();
    }

    /**
     * @brief Retrieves the list of contacts managed by the contact manager.
     * @return returns the list of contacts managed by the contact manager.
     */
    b3Contact* get_contact_list() {
        return m_contact_manager.get_contact_list();
    }

    /**
     * @brief Clears the world, deallocating all bodies and associated resources.
     */
    void clear();

    /**
     * @brief Sets a flag to indicate that new contacts need to be checked.
     */
    inline void awake_contact_check() {
        m_new_contacts = true;
    }

    /**
     * @brief Provides access to the block allocator (m_block_allocator) used by the b3World.
     * @return Pointer to b3BlockAllocator, pointing to the block allocator used by the world.
     */
    inline b3BlockAllocator* get_block_allocator() {
        return &m_block_allocator;
    }

    /**
     * @brief Retrieves the count of bodies (m_body_count) currently in the b3World.
     * @return representing the current count of bodies in the world.
     */
    inline const int32& get_body_count() const {
        return m_body_count;
    }

    /**
     * @brief Retrieves the list of bodies in the world.
     * @return returns the list of bodies in the world.
     */
    inline b3Body* get_body_list() {
        return m_body_list;
    }

    /**
     * @brief Sets the debug drawing interface for the world.
     * @param draw Pointer to a b3Draw object (draw), which handles debug drawing.
     */
    inline void set_debug_draw(b3Draw* draw) {
        m_debug_draw = draw;
    }

    /**
     * @brief Sets a contact listener for handling contact events.
     * @param listener A pointer to a b3ContactListener object that will handle contact events.
     */
    inline void set_contact_listener(b3ContactListener* listener) {
        m_contact_manager.m_contact_listener = listener;
    }

    /**
     * @brief Sets whether continuous physics simulation is enabled.
     * @param flag Boolean parameter (flag) indicating whether continuous physics should
     * be enabled (true) or disabled (false).
     */
    inline void set_continuous_physics(bool flag) {
        m_continuous_physics = flag;
    }

    /**
     * @brief Retrieves the current state of continuous physics simulation.
     * @return Boolean value indicating whether continuous physics is enabled (true)
     * or disabled (false).
     */
    inline bool get_continuous_physics() const {
        return m_continuous_physics;
    }

private:

    /**
     * @brief Solves constraints and interactions between bodies for a given time step.
     * @param step  A reference to a b3TimeStep object that contains time step information
     * for the simulation
     */
    void solve(b3TimeStep& step);

    /**
     * @brief Draws a shape for debug visualization.
     * @param fixture Pointer to the fixture containing the shape to draw.
     * @param xf Transformation (position and orientation) of the shape.
     * @param color Color to use for drawing the shape.
     */
    void draw_shape(b3Fixture* fixture, const b3Transr& xf, const b3Color& color);
};


#endif //BOX3D_B3_WORLD_HPP
