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


#ifndef BOX3D_B3_CUBE_SPHERE_CONTACT_HPP
#define BOX3D_B3_CUBE_SPHERE_CONTACT_HPP

#include "collision/b3_contact.hpp"

/**
 * @brief Used to deal with collision detection between cubes and spheres
 */
class b3CubeAndSphereContact : public b3Contact {

public:


    /**
     * @brief Create a b3CubeAndSphereContact object
     * @param fixture_a Pointer to the first fixture (cube)
     * @param index_a Index of the object related to the collision in the first fixture
     * @param fixture_b Pointer to the second fixture (sphere)
     * @param index_b Index of the object related to the collision in the second fixture
     * @param block_allocator Pointer to the memory allocator,
     * used to allocate the memory space required for the collision object
     * @return Returns a pointer to the `b3CubeAndSphereContact` object
     */
    static b3Contact* create(b3Fixture* fixture_a, int32 index_a, b3Fixture* fixture_b, int32 index_b, b3BlockAllocator* block_allocator);

    /**
     * @brief Destroys a `b3CubeAndSphereContact` object and frees the memory it occupies
     * @param contact A pointer to the `b3Contact` object to be destroyed
     * @param block_allocator A memory allocator pointer used for managing and releasing memory
     */
    static void destroy(b3Contact* contact, b3BlockAllocator* block_allocator);

    /**
     * @brief Constructor of b3CubeAndSphereContact
     * @param fixture_a The first fixture (cube) associated with the collision object
     * @param fixture_b The second fixture (sphere) associated with the collision object.
     */
    b3CubeAndSphereContact(b3Fixture* fixture_a, b3Fixture* fixture_b);

    /**
     * @brief Virtual destructor of the b3CubeAndSphereContact class.
     */
    virtual ~b3CubeAndSphereContact() = default;

    /**
     * @brief Evaluate collision between a cube and a sphere, storing collision
     * information in the provided collision data structure (`b3Manifold`).
     * @param manifold The data structure used to store collision information
     * @param xf_a Transformation information of the first cube
     * @param xf_b Transformation information of the second sphere
     */
    void evaluate(b3Manifold* manifold, const b3Transr& xf_a, const b3Transr& xf_b) override;
};


#endif // BOX3D_B3_CUBE_SPHERE_CONTACT_HPP