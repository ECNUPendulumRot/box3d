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


#include "collision/b3_cube_contact.hpp"
#include "collision/b3_fixture.hpp"
#include "collision/b3_collision.hpp"

/**
 * @brief Create a collision object of type 'b3CubeContact'.
 * @param fixture_a Pointer to the first fixture
 * @param index_a Index of the object associated with the collision in the first fixture
 * @param fixture_b Pointer to the second fixture
 * @param index_b Index of the object associated with the collision in the second fixture
 * @param block_allocator Pointer to the memory allocator used for allocating memory
 * space required by the collision object
 * @return Return a pointer to the newly created object.
 */
b3Contact *b3CubeContact::create(
    b3Fixture *fixture_a, int32 index_a,
    b3Fixture *fixture_b, int32 index_b,
    b3BlockAllocator *block_allocator) {
    void *mem = block_allocator->allocate(sizeof(b3CubeContact));
    return new (mem) b3CubeContact(fixture_a, fixture_b);
}

/**
 * @brief Destroy a b3CubeContact type collision object and free the memory it occupies.
 * @param contact A pointer to the b3Contact object to be destroyed.
 * @param block_allocator Used for managing and freeing memory.
 */
void b3CubeContact::destroy(b3Contact *contact, b3BlockAllocator *block_allocator) {
    ((b3CubeContact *)contact)->~b3CubeContact();
    block_allocator->free(contact, sizeof(b3CubeContact));
}

/**
 * @brief Constructor of b3CubeContact
 * @param fixture_a Represents the first fixture associated with the collision object.
 * @param fixture_b Represents the second fixture associated with the collision object.
 */
b3CubeContact::b3CubeContact(b3Fixture *fixture_a, b3Fixture *fixture_b) :
    b3Contact(fixture_a, 0, fixture_b, 0) {
    b3_assert(m_fixture_a->get_shape_type() == b3ShapeType::e_cube);
    b3_assert(m_fixture_b->get_shape_type() == b3ShapeType::e_cube);
}

/**
 * @brief Evaluate collision between two cube shapes and store collision
 * information in the provided collision data structure (b3Manifold).
 * @param manifold A data structure used to store collision information.
 * @param xf_a Transformation information of the first cube.
 * @param xf_b Transformation information of the second cube.
 */
void b3CubeContact::evaluate(b3Manifold *manifold, 
							 const b3Transr &xf_A,
							 const b3Transr &xf_B) {
    b3_collide_cube(manifold, (b3CubeShape *)m_fixture_a->get_shape(), xf_A,
				    (b3CubeShape *)m_fixture_b->get_shape(), xf_B);
}
