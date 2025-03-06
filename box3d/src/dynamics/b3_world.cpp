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


#include "dynamics/b3_world.hpp"

#include "common/b3_time_step.hpp"

#include "collision/b3_fixture.hpp"
#include "collision/b3_contact.hpp"

#include "solver/b3_solver.hpp"
#include "solver/b3_solver_zhb.hpp"
#include "common/b3_draw.hpp"

#include "geometry/b3_cube_shape.hpp"
#include "geometry/b3_plane_shape.hpp"
#include "geometry/b3_sphere_shape.hpp"
#include "solver/b3_solver_gr.hpp"

/**
 * @brief Constructor to initialize a world with default settings.
 */
b3World::b3World():
    m_body_list(nullptr), m_body_count(0),
    m_shape_list(nullptr), m_shape_count(0)
{
    m_contact_manager.set_block_allocator(&m_block_allocator);
}

/**
 * @brief Constructor to initialize a world with a specified gravity vector.
 * @param gravity Vector representing the gravitational force acting on bodies.
 */
b3World::b3World(const b3Vec3r &gravity):b3World()
{
    m_gravity = gravity;
    m_contact_manager.set_block_allocator(&m_block_allocator);
}

/**
 * @brief Destructor of b3World
 */
b3World::~b3World()
{
    // TODO: think about how to destruct
}

/**
 * @brief Creates a new body in the world based on the provided body definition.
 * @param def Definition of the body to be created.
 * @return Pointer to the created body.
 */
b3Body *b3World::create_body(const b3BodyDef &def)
{
    b3_assert(def.m_type != b3BodyType::b3_type_not_defined);

    // allocate memory for the body
    void *mem = m_block_allocator.allocate(sizeof(b3Body));
    auto *body = new (mem) b3Body(def);

    body->set_world(this);
    body->apply_gravity(m_gravity);

    // add to the double linked list
    body->m_prev = nullptr;
    body->m_next = this->m_body_list;

    if (m_body_list) {
  	    m_body_list->m_prev = body;
    }
    m_body_list = body;
    ++m_body_count;

    return body;
}

/**
 * @brief Adds a shape to the world
 * @param shape Pointer to the shape to add.
 */
void b3World::add_shape(b3Shape* shape)
{
    shape->set_next(m_shape_list);
    m_shape_list = shape;
    m_shape_count++;
}

/**
 * @brief Clears the world, deallocating all bodies and associated resources.
 */
void b3World::clear()
{
    // TODO: Check this function
    // Free all bodies
    b3Body* body = m_body_list;
    while (body != nullptr) {
        auto* next = body->next();
        // b3_free(body);
        body->destroy_fixtures();
        // TODO: free contact and contact edge etc related to body (destroy a body)
        m_block_allocator.free(body, sizeof(b3Body));
        body = next;
    }
}

/**
 * @brief Advances the physics simulation by one time step.
 * @param dt Time step size.
 * @param velocity_iterations Iterations for velocity updates.
 * @param position_iterations Iterations for position updates.
 */
void b3World::step(real dt, int32 velocity_iterations, int32 position_iterations)
{

    // when new fixtures were added, we need to find the new contacts.
    if (m_new_contacts) {
        m_contact_manager.find_new_contact();
        m_new_contacts = false;
    }

    b3TimeStep step;
    step.m_dt = dt;
    step.m_velocity_iterations = velocity_iterations;
    step.m_position_iterations = position_iterations;
    step.m_integral_method = e_implicit;
    step.m_inv_dt = dt > 0.0 ? real(1.0) / dt : real(0.0);

    // update contacts, aabb updates, when aabb not overlapping, delete the contact,
    // otherwise, update the contact manifold.
    m_contact_manager.collide();

    // generate islands, and solve them.
    if (m_step_complete && step.m_dt > 0.0) {
        solve(step);
    }

    if (m_continuous_physics && step.m_dt > 0.0) {
        //solve_toi(step);
    }
}

/**
 * @brief Draws debug information about bodies and shapes in the world.
 */
void b3World::debug_draw() {

    if (m_debug_draw == nullptr) {
        return;
    }

    uint32 flags = m_debug_draw->get_flags();

    if (flags & b3Draw::e_shape_bit) {
        for (b3Body *body = m_body_list; body; body = body->next()) {

            b3Transr xf(body->get_position(), body->get_quaternion());

            for (b3Fixture *f = body->get_fixture_list(); f; f = f->get_next()) {
                b3Color c;
                if (f->get_body()->is_awake()) {
                    c = b3Color(1.0f, 1.0f, 0.0f);
                } else {
                    c = b3Color(0.5f, 0.5f, 0.5f);
                }
                draw_shape(f, xf, c);
            }
        }
    }
}

/**
 * @brief Draws a shape for debug visualization.
 * @param fixture Pointer to the fixture containing the shape to draw.
 * @param xf Transformation (position and orientation) of the shape.
 * @param color Color to use for drawing the shape.
 */
void b3World::draw_shape(b3Fixture *fixture, const b3Transr&xf, const b3Color &color) {

    switch (fixture->get_shape_type()) {
        case b3ShapeType::e_cube: {
            b3CubeShape* cube = (b3CubeShape*)fixture->get_shape();
            m_debug_draw->draw_box(cube, xf, color);
            break;
        }
        case b3ShapeType::e_plane: {
            b3PlaneShape* plane = (b3PlaneShape*)fixture->get_shape();
            m_debug_draw->draw_plane(plane, xf, color);
            break;
        }

        case b3ShapeType::e_sphere: {
            b3SphereShape* sphere = (b3SphereShape*)fixture->get_shape();
            m_debug_draw->draw_sphere(sphere, xf, color);
            break;
        }
        default:
            break;
    }
}

/**
 * @brief Sets whether bodies are allowed to enter sleep mode.
 * @param flag true to allow sleeping, false to disable sleeping.
 */
void b3World::set_allow_sleeping(bool flag) {

    if (flag == m_allow_sleep) {
        return;
    }

    m_allow_sleep = flag;
    if (m_allow_sleep == false) {
        for (b3Body* b = m_body_list; b; b = b->m_next) {
            b->set_awake(true);
        }
    }
}


/**
 * @brief Solves constraints and interactions between bodies for a given time step.
 * @param step  A reference to a b3TimeStep object that contains time step information
 * for the simulation
 */
void b3World::solve(b3TimeStep& step)
{
    // clear all island flag.
    for (b3Body *body = m_body_list; body; body = body->next()) {
        body->m_flags &= ~b3Body::e_island_flag;
    }
    for (b3Contact *contact = m_contact_manager.get_contact_list(); contact; contact = contact->next()) {
        contact->unset_flag(b3Contact::e_island_flag);
    }


    b3Island island(&m_block_allocator, m_body_count,
                    m_contact_manager.get_contact_count());

    // build all islands
    void* mem = m_block_allocator.allocate(m_body_count * sizeof(b3Body *));
    b3Body **stack = new (mem) b3Body *;
    for (b3Body *body = m_body_list; body; body = body->next()) {

        if (body->m_flags & b3Body::e_island_flag) {
            continue;
        }

        if (body->is_awake() == false) {
            continue;
        }

        if (body->get_type() == b3BodyType::b3_static_body) {
            continue;
        }

        int32 stack_count = 0;
        stack[stack_count++] = body;
        body->m_flags |= b3Body::e_island_flag;

        // Perform a depth first search (DFS) on the constraint graph.
        while (stack_count > 0) {
            b3Body *b = stack[--stack_count];

            island.add_body(b);

            // do not propagate islands across static bodies
            if (b->get_type() == b3BodyType::b3_static_body) {
                continue;
            }

            // search all contact connected to this body
            for (b3ContactEdge *ce = b->get_contact_list(); ce; ce = ce->m_next) {
                b3Contact *contact = ce->m_contact;

                // Has this contact already has been added to this island ?
                if (contact->test_flag(b3Contact::e_island_flag)) {
                    continue;
                }
                if (!contact->test_flag(b3Contact::e_touching_flag)) {
                    continue;
                }

                island.add_contact(contact);
                contact->set_flag(b3Contact::e_island_flag);

                b3Body *other = ce->m_other;

                // Was the other body already has been added to this island ?
                if (other->m_flags & b3Body::e_island_flag) {
                    continue;
                }

                stack[stack_count++] = other;
                other->m_flags |= b3Body::e_island_flag;
            }
        }

        // solve the constraints
        //b3Solver solver(&m_block_allocator, &island, &step);
        b3Solver solver(&m_block_allocator, &island, &step);
        solver.solve(m_allow_sleep);
        // Post solve cleanup.

        for(int32 i = 0; i < island.get_body_count(); ++i) {
            // Allow static bodies to participate in other islands
            b3Body* body = island.get_body(i);
            if(body->m_type == b3BodyType::b3_static_body) {
                body->m_flags &= ~b3Body::e_island_flag;
            }
        }
        // clear all bodies and contacts count, so we can reuse the island for the next island.
        island.clear();
    }

    // Free the stack and island memory.
    m_block_allocator.free(stack, m_body_count * sizeof(b3Body*));

    // TODO: synchronize ?
    for (b3Body *b = m_body_list; b; b = b->m_next) {
        // If a body was not in an island then it did not move.
        if ((b->m_flags & b3Body::e_island_flag) == 0) {
            continue;
        }
        if (b->m_type == b3BodyType::b3_static_body) {
            continue;
        }

        // Update fixtures(for broad-phase).
        b->synchronize_fixtures();
    }
    // Look for new contacts
    m_contact_manager.find_new_contact();
}

