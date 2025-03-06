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


#include "geometry/b3_cube_shape.hpp"

#include "common/b3_common.hpp"
#include "common/b3_block_allocator.hpp"

#include "dynamics/b3_mass_property.hpp"
#include "dynamics/b3_body.hpp"


#include "collision/b3_aabb.hpp"

/**
 * @brief the constructor of b3CubeShape
 */
b3CubeShape::b3CubeShape() {
    m_radius = b3_polygon_radius;
    m_type = e_cube;
}

/**
 * @brief create the cube with length, width and height
 * @param hx: the half width of the cube
 * @param hy: the half length of the cube
 * @param hz: the half height of the cube
 */
void b3CubeShape::set_as_box(double hx, double hy, double hz) {
    b3_assert(hx > 0.0f);
    b3_assert(hy > 0.0f);
    b3_assert(hz > 0.0f);

    m_centroid.set_zero();

    // TODO: check the order of the axis
    m_vertices[0].set(hx, -hy, -hz);
    m_vertices[1].set(hx, hy, -hz);
    m_vertices[2].set(-hx, hy, -hz);
    m_vertices[3].set(-hx, -hy, -hz);

    m_vertices[4].set(hx, -hy, hz);
    m_vertices[5].set(hx, hy, hz);
    m_vertices[6].set(-hx, hy, hz);
    m_vertices[7].set(-hx, -hy, hz);

    m_normals[0].set(0.0f, 0.0f, -1.0f);
    m_normals[1].set(0.0f, 1.0f, 0.0f);
    m_normals[2].set(0.0f, 0.0f, 1.0f);
    m_normals[3].set(0.0f, -1.0f, 0.0f);
    m_normals[4].set(1.0f, 0.0f, 0.0f);
    m_normals[5].set(-1.0f, 0.0f, 0.0f);

    m_faces[0] = { 0, 3, 2, 1 };
    m_faces[1] = { 1, 2, 6, 5 };
    m_faces[2] = { 4, 5, 6, 7 };
    m_faces[3] = { 0, 4, 7, 3 };
    m_faces[4] = { 0, 1, 5, 4 };
    m_faces[5] = { 2, 3, 7, 6 };

    m_edges[0] = { 0, 1 };
    m_edges[1] = { 1, 2 };
    m_edges[2] = { 2, 3 };
    m_edges[3] = { 3, 0 };
    m_edges[4] = { 4, 5 };
    m_edges[5] = { 5, 6 };
    m_edges[6] = { 6, 7 };
    m_edges[7] = { 7, 4 };
    m_edges[8] = { 0, 4 };
    m_edges[9] = { 1, 5 };
    m_edges[10] = { 2, 6 };
    m_edges[11] = { 3, 7 };

    m_xyz.set(2 * hx, 2 * hy, 2 * hz);
    m_h_xyz.set(hx, hy, hz);

}

void b3CubeShape::set_as_box(double hx, double hy, double hz, b3Vec3r center, b3Vec3r angle) {
    b3_assert(hx > 0.0f);
    b3_assert(hy > 0.0f);
    b3_assert(hz > 0.0f);

    m_centroid = center;

    // TODO: check the order of the axis
    m_vertices[0].set(hx, -hy, -hz);
    m_vertices[1].set(hx, hy, -hz);
    m_vertices[2].set(-hx, hy, -hz);
    m_vertices[3].set(-hx, -hy, -hz);

    m_vertices[4].set(hx, -hy, hz);
    m_vertices[5].set(hx, hy, hz);
    m_vertices[6].set(-hx, hy, hz);
    m_vertices[7].set(-hx, -hy, hz);

    m_normals[0].set(0.0f, 0.0f, -1.0f);
    m_normals[1].set(0.0f, 1.0f, 0.0f);
    m_normals[2].set(0.0f, 0.0f, 1.0f);
    m_normals[3].set(0.0f, -1.0f, 0.0f);
    m_normals[4].set(1.0f, 0.0f, 0.0f);
    m_normals[5].set(-1.0f, 0.0f, 0.0f);

    m_faces[0] = { 0, 3, 2, 1 };
    m_faces[1] = { 1, 2, 6, 5 };
    m_faces[2] = { 4, 5, 6, 7 };
    m_faces[3] = { 0, 4, 7, 3 };
    m_faces[4] = { 0, 1, 5, 4 };
    m_faces[5] = { 2, 3, 7, 6 };

    m_edges[0] = { 0, 1 };
    m_edges[1] = { 1, 2 };
    m_edges[2] = { 2, 3 };
    m_edges[3] = { 3, 0 };
    m_edges[4] = { 4, 5 };
    m_edges[5] = { 5, 6 };
    m_edges[6] = { 6, 7 };
    m_edges[7] = { 7, 4 };
    m_edges[8] = { 0, 4 };
    m_edges[9] = { 1, 5 };
    m_edges[10] = { 2, 6 };
    m_edges[11] = { 3, 7 };

    m_xyz.set(2 * hx, 2 * hy, 2 * hz);
    m_h_xyz.set(hx, hy, hz);

    b3Transr xf;
    xf.m_p = center;
    b3Quatr q(angle);
    xf.m_r = q.rotation_matrix();

    for(int32 i=0;i<8;i++){
        m_vertices[i] = xf.transform(m_vertices[i]);
    }

    for(int32 i=0;i<6;i++){
        m_normals[i] = xf.rotate(m_normals[i]);
    }
}
/**
 * @brief compute the axis-aligned bounding box (AABB) for the cube shape
 * @param aabb A pointer to a b3AABB object where the resulting AABB will be stored.
 * @param xf A constant reference to a b3Transr object representing the transformation to apply to the cube.
 * @param childIndex An integer representing the index of the child shape. Not used
 * in this function as a cube is considered a single shape.
 */
void b3CubeShape::get_bound_aabb(b3AABB *aabb, const b3Transr &xf, int32 childIndex) const {
    b3_NOT_USED(childIndex);

    b3Vec3r min = xf.transform(m_vertices[0]);
    b3Vec3r max = min;

    for (int32 i = 1; i < 8; ++i) {
  	    b3Vec3r v = xf.transform(m_vertices[i]);
  	    min = b3_min_coeff(min, v);
  	    max = b3_max_coeff(max, v);
    }

    b3Vec3r r(m_radius, m_radius, m_radius);
    aabb->m_min = min - r;
    aabb->m_max = max + r;
}

/**
 * @brief calculate and set the mass properties of the cube shape, given its density.
 * @param mass_data A reference to a b3MassProperty object where the calculated mass
 * properties will be stored.
 * @param density A real number representing the density of the material of the cube.
 */
void b3CubeShape::compute_mass_properties(b3MassProperty &mass_data, real density) const {
    mass_data.m_center = m_centroid;

    mass_data.m_volume = m_xyz.x * m_xyz.y * m_xyz.z;

    mass_data.m_mass = density * mass_data.m_volume;

    real x2 = m_xyz.x * m_xyz.x;
    real y2 = m_xyz.y * m_xyz.y;
    real z2 = m_xyz.z * m_xyz.z;
    real ot = 1.0 / 12.0;

    real i11 = ot * mass_data.m_mass * (y2 + z2);
    real i22 = ot * mass_data.m_mass * (x2 + z2);
    real i33 = ot * mass_data.m_mass * (x2 + y2);

    mass_data.m_Inertia = b3Mat33r::zero();
    mass_data.m_Inertia(0, 0) = i11;
    mass_data.m_Inertia(1, 1) = i22;
    mass_data.m_Inertia(2, 2) = i33;
}

/**
 * @brief create and return a deep copy of the current b3CubeShape object.
 * @return return a deep copy of the current b3CubeShape object.
 */
b3Shape *b3CubeShape::clone() const {
    void *mem = m_block_allocator->allocate(sizeof(b3CubeShape));
    auto *clone = new (mem) b3CubeShape;
    *clone = *this;
    return clone;
}



