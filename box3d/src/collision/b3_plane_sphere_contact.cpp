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



#include "collision/b3_plane_sphere_contact.hpp"
#include "collision/b3_fixture.hpp"
#include "collision/b3_collision.hpp"

/**
 * @brief A static method to create a b3PlaneSphereContact object.
 * @param fixture_a Pointer to the first fixture (plane).
 * @param index_a Index related to fixture A
 * @param fixture_b Pointer to the second fixture (sphere).
 * @param index_b Index related to fixture B
 * @param block_allocator Pointer to the block allocator for memory management.
 * @return A pointer to the created b3PlaneSphereContact object.
 */
b3Contact* b3PlaneSphereContact::create(
    b3Fixture *fixture_a, int32 index_a,
    b3Fixture *fixture_b, int32 index_b,
    b3BlockAllocator* block_allocator)
{
    void* mem = block_allocator->allocate(sizeof(b3PlaneSphereContact));
    return new (mem) b3PlaneSphereContact(fixture_a, fixture_b);
}

/**
 * @brief A static method to destroy a b3PlaneSphereContact object and free its allocated memory.
 * @param contact Pointer to the contact object to destroy.
 * @param block_allocator Pointer to the block allocator for memory management.
 */
void b3PlaneSphereContact::destroy(b3Contact *contact, b3BlockAllocator *block_allocator)
{
    ((b3PlaneSphereContact*)contact)->~b3PlaneSphereContact();
    block_allocator->free(contact, sizeof(b3PlaneSphereContact));
}

/**
 * @brief  Constructor that initializes the contact with the given fixtures.
 * @param fixture_a Pointer to the first fixture (plane).
 * @param fixture_b Pointer to the second fixture (sphere).
 */
b3PlaneSphereContact::b3PlaneSphereContact(b3Fixture *fixture_a, b3Fixture *fixture_b):
    b3Contact(fixture_a, 0, fixture_b, 0)
{
    b3_assert(fixture_a->get_shape_type() == e_plane);
    b3_assert(fixture_b->get_shape_type() == e_sphere);
}

/**
 * @brief Method to evaluate and compute the collision manifold between the plane and sphere.
 * @param manifold Pointer to the manifold structure where the collision information is stored.
 * @param xf_a Transform of the first fixture (plane).
 * @param xf_b Transform of the second fixture (sphere).
 */
void b3PlaneSphereContact::evaluate(b3Manifold *manifold, const b3Transr &xf_a, const b3Transr &xf_b)
{
    b3_collide_plane_and_sphere(
        manifold,
        (b3PlaneShape*)get_fixture_a()->get_shape(), xf_a,
        (b3SphereShape*)get_fixture_b()->get_shape(), xf_b
    );
}