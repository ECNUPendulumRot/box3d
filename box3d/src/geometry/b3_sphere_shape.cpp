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


#include "geometry/b3_sphere_shape.hpp"

#include <memory>

#include "common/b3_allocator.hpp"
#include "common/b3_block_allocator.hpp"

#include "dynamics/b3_mass_property.hpp"
#include "dynamics/b3_body.hpp"

#include "collision/b3_aabb.hpp"

/**
 * @brief constructor of b3SphereShape
 */
b3SphereShape::b3SphereShape()
{
    m_radius = 0;
    m_type = e_sphere;
}

/**
 * @brief when create a sphere, we need call this function at first.
 * @param radius
 */
void b3SphereShape::set_as_sphere(real radius)
{
    m_radius = radius;
    m_centroid.set_zero();
}

/**
 * @brief Calculates the Axis-Aligned Bounding Box (AABB) for the sphere.
 * @param aabb A pointer to an AABB object that will store the calculated bounding box.
 * @param xf A transformation object representing the sphere's position and orientation.
 * @param child_index The index of the child shape
 */
void b3SphereShape::get_bound_aabb(b3AABB *aabb, const b3Transr& xf, int32 child_index) const
{
    b3_NOT_USED(child_index);

    b3Vec3r centroid = xf.transform(m_centroid);

    b3Vec3r radius(m_radius, m_radius, m_radius);

    aabb->m_min = centroid - radius;
    aabb->m_max = centroid + radius;
}

/**
 * @brief Computes the mass properties of the sphere
 * @param mass_data A reference to a b3MassProperty object that will store the
 * calculated mass properties.
 * @param density The density of the material.
 */
void b3SphereShape::compute_mass_properties(b3MassProperty& mass_data, real density) const
{
    mass_data.m_center = m_centroid;

    mass_data.m_volume = 4 * real(b3_pi) * m_radius * m_radius * m_radius / 3;

    mass_data.m_mass = density * mass_data.m_volume;

    real ixx = real(0.4) * mass_data.m_mass * m_radius * m_radius;

    mass_data.m_Inertia = b3Mat33r::zero();
    mass_data.m_Inertia(0, 0) = ixx;
    mass_data.m_Inertia(1, 1) = ixx;
    mass_data.m_Inertia(2, 2) = ixx;
}

/**
 * @brief Creates a copy of the b3SphereShape object.
 * @return A pointer to the new b3SphereShape instance.
 */
b3Shape* b3SphereShape::clone() const
{
    void *memory = m_block_allocator->allocate(sizeof(b3SphereShape));
    auto *clone = new (memory) b3SphereShape;
    *clone = *this;
    return clone;
}

