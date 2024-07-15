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


#include "collision/b3_collision.hpp"
#include "geometry/b3_plane_shape.hpp"
#include "geometry/b3_sphere_shape.hpp"
#include "geometry/b3_cube_shape.hpp"
#include "math/b3_math_op.hpp"


// project the box onto a separation axis called axis
// the axis is in the world frame
/**
 * @brief The function projects a given box (b3CubeShape) onto a specified
 * separation axis. The separation axis is provided in the world frame.
 * @param box A reference to the box shape which is to be projected.
 * @param xf The transformation of the box.
 * @param axis The separation axis in the world frame onto which the box is projected.
 * @return : The scalar projection of the box onto the given axis,
 * representing the extent of the box along that axis.
 */
static real transform_to_axis(const b3CubeShape& box, const b3Transr& xf, const b3Vec3r& axis)
{
    const b3Mat33r& R = xf.m_r;
    const b3Vec3r& half_xyz = box.m_h_xyz;

    return half_xyz.x * b3_abs(axis.dot(R.col(0))) + half_xyz.y * b3_abs(axis.dot(R.col(1))) + half_xyz.z * b3_abs(axis.dot(R.col(2)));
}


// check whether two box will overlap under selected axis
// separate cube_B from cube_A
// so the axis is also from cube_A
/**
 * @brief The function checks whether two boxes overlap along a given axis.
 * It calculates the penetration depth of one box relative to the other
 * along the specified axis.
 * @param xf_A_plane The transformation of the reference box cube_A.
 * @param cube_B The shape of the second box which is being checked for overlap.
 * @param xf_B The transformation of cube_B.
 * @param axis The axis along which the overlap is being checked.
 * @param penetration A reference variable to store the penetration depth.
 */
static void overlap_on_axis(
    const b3Transr& xf_A_plane,
    const b3CubeShape& cube_B, const b3Transr& xf_B,
    const b3Vec3r& axis, real& penetration)
{
    // project box onto the axis
    real project_B = transform_to_axis(cube_B, xf_B, axis);

    const b3Vec3r& plane_center = xf_A_plane.position();
    b3Vec3r to_center = xf_B.position() - plane_center;

    // set the penetration of current axis and return whether overlapped.
    // penetration is a value that is smaller than zero
    // the larger the value is, the smaller the penetration is.
    real r = to_center.dot(axis);
    penetration = r - project_B;
}

/**
 * @brief Compute the collision manifold between a plane and a sphere.
 * @param manifold Pointer to the b3Manifold structure for storing details of collisions between the plane and the sphere
 * @param plane_a A pointer to the shape of the plane involved in the collision
 * @param xf_a Information about the transformation of the plane, including position and direction
 * @param sphere_b A pointer to the shape of the sphere involved in the collision
 * @param xf_b Information about the transformation of the sphere, including position and direction
 */
void b3_collide_plane_and_sphere(
    b3Manifold* manifold,
    const b3PlaneShape* plane_a,
    const b3Transr& xf_a,
    const b3SphereShape* sphere_b,
    const b3Transr& xf_b)
{
    manifold->m_point_count = 0;

    // transform sphere center to plane frame
    b3Vec3r sphere_local_center = xf_b.transform(sphere_b->get_centroid());
    sphere_local_center = xf_a.transform_local(sphere_local_center);

    b3Vec3r closest_point = sphere_local_center;

    closest_point.x = b3_clamp(closest_point.x, -plane_a->m_half_width, plane_a->m_half_width);
    closest_point.y = b3_clamp(closest_point.y, -plane_a->m_half_length, plane_a->m_half_length);
    closest_point.z = 0;

    b3Vec3r normal = sphere_local_center - closest_point;
    real total_radius = sphere_b->get_radius() + plane_a->get_radius();
    if (normal.length2() > total_radius * total_radius) {
        return;
    }

    if (normal.z <= 0) {
        normal = b3Vec3r(0, 0, 1);
    }

    manifold->m_point_count = 1;
    manifold->m_local_point = {0, 0, 0};
    manifold->m_local_normal = normal.normalized();
    manifold->m_points[0].m_local_point = {0, 0, 0};
    manifold->m_type = b3Manifold::e_face_A;
}

/**
 * @brief The function identifies the face of the second cube (cube2) that is most
 * anti-parallel to a given normal vector (n_p).
 * @param c An array to store the vertices of the incident face.
 * @param n_p A normal vector in the world frame.
 * @param cube2 A pointer to the second cube's shape data.
 * @param xf2 The transformation of the second cube.
 * @return The index of the incident face on cube2.
 */
static int32 find_incident_face(
    b3ClipVertex c[4],
    const b3Vec3r & n_p,
    const b3CubeShape* cube2, const b3Transr& xf2)
{
    int32 count2 = 6;
    const b3Vec3r* normals2 = cube2->m_normals;
    const b3Vec3r* vertices2 = cube2->m_vertices;
    const b3EdgeIndex* edges2 = cube2->m_edges;
    const b3FaceIndex* faces2 = cube2->m_faces;

    // Transform the normal from the world frame to the local frame of cube2
    const b3Vec3r& n_p_b = xf2.rotation_matrix().transpose() * n_p;

    // find the incident face on cube2
    // the incident face is the mose anti-parallel face of cube2 to face1
    // so the dot product is the least
    int32 incident_face_index_2 = 0;
    real min_dot = b3_real_max;
    for (int32 i = 0; i < count2; ++i) {
        real dot = n_p_b.dot(normals2[i]);
        if (dot < min_dot) {
            min_dot = dot;
            incident_face_index_2 = i;
        }
    }

    b3FaceIndex incident_face = faces2[incident_face_index_2];

    for (int32 i = 0; i < 4; ++i) {
        c[i].v = vertices2[edges2[incident_face.e[i]].v1];
        c[i].id.cf.type = b3ContactFeature::e_f_p;
        c[i].id.cf.index_1 = (uint8)0;
        c[i].id.cf.index_2 = (uint8)edges2[incident_face.e[i]].v1;
        c[i].id.cf.index_ext  = (uint8)incident_face.e[i];
    }

    return incident_face_index_2;
}

/**
 * @brief Compute the collision manifold between a plane and a cube.
 * @param manifold Pointer to the b3Manifold structure for storing details of collisions between planes and cubes
 * @param plane_a A pointer to the shape of the plane involved in the collision
 * @param xf_a Information about the transformation of the plane, including position and direction
 * @param cube_b A pointer to the cube shape involved in the collision
 * @param xf_b The transformation information of the cube, including position and orientation
 */
void b3_collide_plane_and_cube(
    b3Manifold* manifold,
    const b3PlaneShape* plane_a,
    const b3Transr& xf_a,
    const b3CubeShape* cube_b,
    const b3Transr& xf_b)
{
    manifold->m_point_count = 0;

    real total_radius = cube_b->get_radius() + plane_a->get_radius();

    const b3Vec3r& axis = xf_a.rotation_matrix().col(2);

    real penetration;

    overlap_on_axis(xf_a, *cube_b, xf_b, axis, penetration);

    if (penetration > total_radius) {
        return;
    }

    // the equation of the plane is n * x - n * t = 0;
    // n * t is called front_offset
    real front_offset = axis.dot(xf_a.position());

    b3ClipVertex incident_vertices[4];
    find_incident_face(incident_vertices, axis, cube_b, xf_b);

    int32 point_count = 0;
    for (int32 i = 0; i < 4; ++i) {
        real separation = axis.dot(incident_vertices[i].v) - front_offset;
        if (separation <= total_radius) {
            b3ManifoldPoint* cp = manifold->m_points + point_count;
            b3Vec3r& v = incident_vertices[i].v;
            cp->m_local_point = v;
            cp->id = incident_vertices[i].id;
            ++point_count;
        }
    }
    manifold->m_local_point = {0, 0, 0};
    manifold->m_point_count = point_count;
    manifold->m_local_normal = {0, 0, 1};
    manifold->m_type = b3Manifold::e_face_A;
}