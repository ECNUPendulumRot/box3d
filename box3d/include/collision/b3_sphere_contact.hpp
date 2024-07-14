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


#ifndef BOX3D_B3SPHERECONTACT_HPP
#define BOX3D_B3SPHERECONTACT_HPP

#include "collision/b3_contact.hpp"

/**
 * @briefThe b3SphereContact class handles the contact interactions between two
 * sphere shapes within a physics simulation. It is responsible for creating,
 * destroying, and evaluating the collisions between two spheres.
 */
class b3SphereContact : public b3Contact {

public:

    /**
     * @brief A static method to create an instance of b3SphereContact.
     * @param fixture_a Pointer to the first fixture (sphere).
     * @param index_a Index related to fixture A
     * @param fixture_b Pointer to the second fixture (sphere).
     * @param index_b Index related to fixture B
     * @param block_allocator Pointer to the block allocator for memory management.
     * @return A pointer to the created b3SphereContact object.
     */
    static b3Contact* create(b3Fixture* fixture_a, int32 index_a, b3Fixture* fixture_b, int32 index_b, b3BlockAllocator* block_allocator);

    /**
     * @brief A static method to destroy an instance of b3SphereContact and free
     * its allocated memory.
     * @param contact Pointer to the contact object to destroy.
     * @param block_allocator Pointer to the block allocator for memory management.
     */
    static void destroy(b3Contact* contact, b3BlockAllocator* block_allocator);

    /**
     * @brief Constructor that initializes the contact with the given fixtures.
     * @param fixture_a Pointer to the first fixture (sphere).
     * @param fixture_b Pointer to the second fixture (sphere).
     */
    b3SphereContact(b3Fixture* fixture_a, b3Fixture* fixture_b);

    /**
     * @brief Virtual destructor for safely destroying instances of b3SphereContact.
     */
    virtual ~b3SphereContact() = default;

    /**
     * @brief Method to evaluate and compute the collision manifold between the
     * two sphere shapes.
     * @param manifold Pointer to the manifold structure where the collision
     * information is stored.
     * @param xf_a Transform of the first fixture (sphere).
     * @param xf_b Transform of the second fixture (sphere).
     */
    void evaluate(b3Manifold* manifold, const b3Transr& xf_a, const b3Transr& xf_b) override;
};



#endif // BOX3D_B3SPHERECONTACT_HPP