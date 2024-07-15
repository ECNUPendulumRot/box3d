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

#include "geometry/b3_plane_shape.hpp"
#include "common/b3_block_allocator.hpp"

#include "dynamics/b3_mass_property.hpp"
#include "dynamics/b3_body.hpp"

#include "collision/b3_aabb.hpp"

/**
 * @brief the constructor of b3PlaneShape
 */
b3PlaneShape::b3PlaneShape()
{
    m_radius = b3_polygon_radius;
    m_type = e_plane;
}

/**
 * @brief Defines the plane's dimensions and calculates its vertices.
 * @param point: a point on this plane
 * @param normal: the normal of this plane
*/
void b3PlaneShape::set_as_plane(real length, real width)
{
    m_half_length = length / 2.0;
    m_half_width = width / 2.0;
    m_vertices[0].set(-m_half_width, -m_half_length, 0);
    m_vertices[1].set(-m_half_width, m_half_length, 0);
    m_vertices[2].set(m_half_width, m_half_length, 0);
    m_vertices[3].set(m_half_width, -m_half_length, 0);
}

/**
 * @brief Computes the axis-aligned bounding box (AABB) for the plane.
 * @param aabb Pointer to the AABB to be computed.
 * @param xf Transformation applied to the plane.
 * @param child_index Index of the child shape.
 */
void b3PlaneShape::get_bound_aabb(b3AABB* aabb, const b3Transr& xf, int32 child_index) const
{
    b3_NOT_USED(child_index);

    b3Vec3r min;
    b3Vec3r max;

    b3Vec3r vertices[4];

    vertices[0].set(-m_half_width, -m_half_length, 0);
    vertices[1].set(-m_half_width, m_half_length, 0);
    vertices[2].set(m_half_width, m_half_length, 0);
    vertices[3].set(m_half_width, -m_half_length, 0);

    for (int32 i = 0; i < 4; i++) {
        b3Vec3r v = xf.transform(vertices[i]);
        min = b3_min_coeff(min, v);
        max = b3_max_coeff(max, v);
    }

    b3Vec3r r(m_radius, m_radius, m_radius);
    aabb->m_min = min - r;
    aabb->m_max = max + r;
}

/**
 * @brief Calculates the mass properties of the plane. Since a plane is a static object,
 * its mass and inertia are zero.
 * @param mass_data Reference to the mass properties to be computed.
 * @param density Density of the plane.
 */
void b3PlaneShape::compute_mass_properties(b3MassProperty &mass_data, real density) const
{
    // plane is static object
    mass_data.m_center = b3Vec3r::zero();
    mass_data.m_volume = 0;
    mass_data.m_mass = 0;
    mass_data.m_Inertia = b3Mat33r::zero();
}

/**
 * @brief Creates and returns a deep copy of the current b3PlaneShape object.
 * @return A pointer to the newly created clone of the b3PlaneShape object.
 */
b3Shape* b3PlaneShape::clone() const
{
    void* mem = m_block_allocator->allocate(sizeof(b3PlaneShape));
    auto* clone = new (mem) b3PlaneShape;
    *clone = *this;
    return clone;
}


