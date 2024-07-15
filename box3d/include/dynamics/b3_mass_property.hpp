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


#ifndef BOX3D_B3_INERTIA_HPP
#define BOX3D_B3_INERTIA_HPP


#include "common/b3_types.hpp"
#include "dynamics/b3_transform.hpp"

/**
 * @brief The struct is used to represent the mass properties of a
 * rigid body in a physics simulation.
 */
struct b3MassProperty {

    /**
     * @brief Represents the volume of the rigid body.
     */
    real m_volume;

    /**
     * @brief Represents the mass of the rigid body.
     */
    real m_mass;

    /**
     * @brief The center of mass of the rigid body.
     * @note The m_center is just the integral part divide the volume
     */
    b3Vec3r m_center;

    /**
     * @brief Represents the inertia tensor of the rigid body.
     */
    b3Mat33r m_Inertia;

};


#endif //BOX3D_B3_INERTIA_HPP
