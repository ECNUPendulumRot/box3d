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


#include "collision/b3_contact.hpp"
#include "collision/b3_fixture.hpp"
#include "collision/b3_sphere_contact.hpp"
#include "collision/b3_cube_contact.hpp"
#include "collision/b3_cube_sphere_contact.hpp"
#include "collision/b3_plane_sphere_contact.hpp"
#include "collision/b3_plane_cube_contact.hpp"

#include "dynamics/b3_body.hpp"

#include "common/b3_world_callback.hpp"


b3ContactRegister b3Contact::s_registers[b3ShapeType::e_type_count][b3ShapeType::e_type_count];  ///< Pointers to collision creation and destruction functions that store different shape combinations
bool b3Contact::s_initialized = false; ///< Used to track whether the collision system has been initialized, initialized value is false.

/**
  * @brief Sets up collision handling methods for different shape combinations
  * in physical simulations, ensuring the generation and management of appropriate
  * collision types during collision detection and resolution processes.
  */
void b3Contact::initialize_registers()
{
    add_type(b3SphereContact::create, b3SphereContact::destroy, b3ShapeType::e_sphere, b3ShapeType::e_sphere);
    add_type(b3CubeContact::create, b3CubeContact::destroy, b3ShapeType::e_cube, b3ShapeType::e_cube);
    add_type(b3CubeAndSphereContact::create, b3CubeAndSphereContact::destroy, b3ShapeType::e_cube, b3ShapeType::e_sphere);
    add_type(b3PlaneSphereContact::create, b3PlaneSphereContact::destroy, b3ShapeType::e_plane, b3ShapeType::e_sphere);
    add_type(b3PlaneCubeContact::create, b3PlaneCubeContact::destroy, b3ShapeType::e_plane, b3ShapeType::e_cube);
}

/**
 * @brief Constructor of the b3Contact class
 * @param f_A Pointer to the first fixture object
 * @param index_A The index of the first fixture in its parent object
 * @param f_B Pointer to the second fixture object
 * @param index_B The index of the second fixture in its parent object
 */
b3Contact::b3Contact(b3Fixture *f_A, int32 index_A, b3Fixture *f_B, int32 index_B)
{
    // TODO: check enable_flags

    m_fixture_a = f_A;
    m_fixture_b = f_B;

    m_index_a = index_A;
    m_index_b = index_B;

    m_prev = nullptr;
    m_next = nullptr;

    m_restitution = 0;
    m_friction = 0;

    m_manifold.m_point_count = 0;

    m_restitution = b3_mix_restitution(m_fixture_a->get_restitution(), m_fixture_b->get_restitution());
    m_friction = b3_mix_friction(m_fixture_a->get_friction(), m_fixture_b->get_friction());
    m_restitution_threshold = b3_mix_restitution_threshold(m_fixture_a->get_restitution_threshold(), m_fixture_b->get_restitution_threshold());
}

/**
 * @brief Associate the given shape combination with their
 * corresponding collision handling functions.
 * @param create_fcn A pointer to the function that creates a collision contact.
 * @param destroy_fcn A pointer to the function that destroys a collision contact.
 * @param type_A The first shape of the type
 * @param type_B The second shape of the type
 */
void b3Contact::add_type(
    b3ContactCreateFcn* create_fcn,
    b3ContactDestroyFcn* destroy_fcn,
    b3ShapeType type_A,
    b3ShapeType type_B)
{
    b3_assert(0 <= type_A && type_A < b3ShapeType::e_type_count);
    b3_assert(0 <= type_B && type_B < b3ShapeType::e_type_count);

    s_registers[type_A][type_B].create_fcn = create_fcn;
    s_registers[type_A][type_B].destroy_fcn = destroy_fcn;
    s_registers[type_A][type_B].primary = true;

    if (type_A != type_B) {
        s_registers[type_B][type_A].create_fcn = create_fcn;
        s_registers[type_B][type_A].destroy_fcn = destroy_fcn;
        s_registers[type_B][type_A].primary = false;
    }
}

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
b3Contact *b3Contact::create(
    b3Fixture *fixture_A, int32 index_A,
    b3Fixture *fixture_B, int32 index_B,
    b3BlockAllocator *block_allocator)
{
    if (!s_initialized) {
	    initialize_registers();
	    s_initialized = true;
    }

    b3ShapeType type_A = fixture_A->get_shape_type();
    b3ShapeType type_B = fixture_B->get_shape_type();

    b3_assert(0 <= type_A && type_A < b3ShapeType::e_type_count);
    b3_assert(0 <= type_B && type_B < b3ShapeType::e_type_count);

    b3ContactCreateFcn *create_fcn = s_registers[type_A][type_B].create_fcn;
    if (create_fcn) {
        if (s_registers[type_A][type_B].primary) {
            return create_fcn(fixture_A, index_A, fixture_B, index_B, block_allocator);
        } else {
            return create_fcn(fixture_B, index_B, fixture_A, index_A, block_allocator);
        }
    } else {
  	    return nullptr;
    }
}

/**
 * @brief Used to destroy the specified 'b3Contact' object
 * and release its occupied memory resources.
 * @param contact A pointer to the `b3Contact` object that needs to be destroyed.
 * @param block_allocator A pointer to the memory allocator, used for managing and releasing memory.
 */
void b3Contact::destroy(b3Contact *contact, b3BlockAllocator *block_allocator)
{
    b3_assert(s_initialized == true);

    b3Fixture *fixture_A = contact->get_fixture_a();
    b3Fixture *fixture_B = contact->get_fixture_b();

    if (contact->m_manifold.m_point_count > 0) {
        fixture_A->get_body()->set_awake(true);
        fixture_B->get_body()->set_awake(true);
    }

    b3ShapeType type_A = fixture_A->get_shape_type();
    b3ShapeType type_B = fixture_B->get_shape_type();

    b3_assert(0 <= type_A && type_A < b3ShapeType::e_type_count);
    b3_assert(0 <= type_B && type_B < b3ShapeType::e_type_count);

    b3ContactDestroyFcn *destroy_fcn = s_registers[type_A][type_B].destroy_fcn;
    destroy_fcn(contact, block_allocator);
}

/**
 * @brief Update the contact manifold and touching status.
 * @param listener The listener is a pointer to the b3ContactListener
 * object for possible preprocessing event notifications
 */
void b3Contact::update(b3ContactListener* listener)
{
    // TODO: add warm start, reuse normal and tangent impulse
    // b3Manifold old_manifold = m_manifold
    bool touching = false;
    bool was_touching = (m_flags & e_touching_flag) == e_touching_flag;
    b3Body *body_a = m_fixture_a->get_body();
    b3Body *body_b = m_fixture_b->get_body();

    b3Transr xf_a(body_a->get_position(), body_a->get_quaternion());
    b3Transr xf_b(body_b->get_position(), body_b->get_quaternion());

    // TODO: add sensor ?
    // generate the manifold between the two shapes
    evaluate(&m_manifold, xf_a, xf_b);
    touching = m_manifold.m_point_count > 0;

    if (touching) {
  	    m_flags |= e_touching_flag;
    } else {
  	    m_flags &= ~e_touching_flag;
    }

    if (touching && listener) {
        listener->pre_solve(this, &m_manifold);
    }
}

/**
   * @brief Retrieve detailed information in the world coordinate system
   * for the collision contact, including collision points and normals
   * @param world_manifold Pointer to a b3WorldManifold object, used to store
   * computed collision information in world coordinates.
   */
void b3Contact::get_world_manifold(b3WorldManifold *world_manifold) const
{
    const b3Body *body_a = m_fixture_a->get_body();
    const b3Body *body_b = m_fixture_b->get_body();

    b3Transr xf_a(body_a->get_position(), body_a->get_quaternion());
    b3Transr xf_b(body_b->get_position(), body_b->get_quaternion());

    world_manifold->initialize(&m_manifold, xf_a, m_fixture_a->get_shape()->get_radius(), xf_b, m_fixture_b->get_shape()->get_radius());
}


