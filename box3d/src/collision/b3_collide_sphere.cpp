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
#include "geometry/b3_sphere_shape.hpp"
#include "geometry/b3_cube_shape.hpp"

/**
 * @brief Compute the collision manifold between two circles.
 * @param manifold A pointer to the b3Manifold structure that will be filled with collision details if the spheres collide
 * @param sphere_a A pointer to the shape of the first sphere involved in the collision
 * @param xf_a Constant reference, transformation of the first sphere (position and direction)
 * @param sphere_b A pointer to the shape of the second sphere involved in the collision
 * @param xf_b Transformation of the second sphere (position and direction)
 */
void b3_collide_spheres(
    b3Manifold* manifold,
    const b3SphereShape* sphere_a,
    const b3Transr& xf_a,
    const b3SphereShape* sphere_b,
    const b3Transr& xf_b)
{
    
    manifold->m_point_count = 0;

    // the center of sphere A and sphere B in world frame.
    b3Vec3r p_A = xf_a.position();
    b3Vec3r p_B = xf_b.position();

    b3Vec3r ab = p_B - p_A;

    real sq_distance = ab.length2();
    real radius = sphere_a->get_radius() + sphere_b->get_radius();

    if (sq_distance > radius * radius) {
        // two spheres are not collide.
        return;
    }

    manifold->m_type = b3Manifold::e_spheres;
    manifold->m_local_point = sphere_a->get_centroid();
    manifold->m_local_normal = ab.normalized();
    manifold->m_point_count = 1;

    manifold->m_points[0].m_local_point = sphere_b->get_centroid();
}

/**
 * @brief It finds the face of the cube that is closest to this point.
 * @param cube_a A pointer to the cube's shape data
 * @param sphere_local_position The position of the point inside the cube in the
 * local coordinate system of the cube.
 * @param normal A reference to a vector that will be set to the normal of the
 * closest face of the cube.
 * @param closest_point A reference to a vector that will be set to the closest
 * point on the surface of the cube to sphere_local_position.
 */
static void find_normal(const b3CubeShape* cube_a,
                        const b3Vec3r& sphere_local_position,
                        b3Vec3r& normal, b3Vec3r& closest_point)
{
    real hf[6] = {cube_a->m_h_xyz.z, cube_a->m_h_xyz.y, cube_a->m_h_xyz.z,
                  cube_a->m_h_xyz.y, cube_a->m_h_xyz.x, cube_a->m_h_xyz.x};

    real min_penetration = b3_real_max;

    for (int32 i = 0; i < 6; i++) {
        b3Vec3r n = cube_a->m_normals[i];
        real h = hf[i];
        real dist = sphere_local_position.dot(n) - h;
        if (dist < min_penetration) {
            min_penetration = dist;
            normal = n;
            closest_point = normal * h;
        }
    }
}

/**
 * @brief Compute the collision manifold between circle and cube
 * @param manifold If the cube and the sphere collide, the structure will be filled with collision details.
 * @param cube_a A pointer to the cube shape involved in the collision
 * @param xf_a Transformation of the cube (position and orientation)
 * @param sphere_b A pointer to the shape of the sphere involved in the collision
 * @param xf_b Transformation of the sphere (position and direction)
 */
void b3_collide_cube_and_sphere(
    b3Manifold* manifold,
    const b3CubeShape* cube_a,
    const b3Transr& xf_a,
    const b3SphereShape* sphere_b,
    const b3Transr& xf_b)
{
    
    manifold->m_point_count = 0;

    // Compute the center of sphere position on cube frame
    b3Vec3r sphere_c_local = xf_b.position();
    sphere_c_local = xf_a.transform_local(sphere_c_local);

    b3Vec3r closest_point = sphere_c_local;

    closest_point.x = b3_clamp(closest_point.x, -cube_a->m_h_xyz.x, cube_a->m_h_xyz.x);
    closest_point.y = b3_clamp(closest_point.y, -cube_a->m_h_xyz.y, cube_a->m_h_xyz.y);
    closest_point.z = b3_clamp(closest_point.z, -cube_a->m_h_xyz.z, cube_a->m_h_xyz.z);

    b3Vec3r normal = sphere_c_local - closest_point;
    if (normal.length2() > sphere_b->get_radius() * sphere_b->get_radius()) {
        return;
    }

    if (normal.length2() < b3_real_epsilon * b3_real_epsilon) {
        find_normal(cube_a, sphere_c_local, normal, closest_point);
    }

    manifold->m_point_count = 1;
    manifold->m_local_normal = normal.normalized();
    manifold->m_local_point = closest_point;
    manifold->m_type = b3Manifold::e_face_A;
    manifold->m_points[0].m_local_point = sphere_b->get_centroid();
}

