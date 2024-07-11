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

/**
 * @brief Initialize the member variables of the b3WorldManifold structure based on
 * the given collision manifold and the transformation information of two shapes.
 * @param manifold Contact point set
 * @param xf_A The transformation of shape A
 * @param radius_A the radius of A
 * @param xf_B The transformation of shape B
 * @param radius_B the radius of B
 */
void b3WorldManifold::initialize(
        const b3Manifold *manifold,
        const b3Transr &xf_A, real radius_A,
        const b3Transr &xf_B, real radius_B)
{
    if (manifold->m_point_count == 0) {
        return;
    }

    switch (manifold->m_type) {
        
        case b3Manifold::e_spheres: {
            normal = b3Vec3r(1, 0, 0);
            b3Vec3r point_A = xf_A.transform(manifold->m_local_point);
            b3Vec3r point_B = xf_B.transform(manifold->m_points[0].m_local_point);

            if ((point_A - point_B).length2() > b3_real_epsilon * b3_real_epsilon) {
                normal = (point_B - point_A).normalized();
            }

            b3Vec3r c_A = point_A + radius_A * normal;
            b3Vec3r c_B = point_B - radius_B * normal;
            points[0] = (c_A + c_B) * real(0.5);
            separations[0] = (c_B - c_A).dot(normal);
        }
        break;
        case b3Manifold::e_face_A: {

            normal = xf_A.rotate(manifold->m_local_normal);
            b3Vec3r plane_point = xf_A.transform(manifold->m_local_point);

            for (int i = 0; i < manifold->m_point_count; ++i) {
                b3Vec3r clip_point = xf_B.transform(manifold->m_points[i].m_local_point);
                b3Vec3r c_A = clip_point + (radius_A - (clip_point - plane_point).dot(normal)) * normal;
                b3Vec3r c_B = clip_point - radius_B * normal;
                points[i] = (c_A + c_B) * real(0.5);
                separations[i] = (c_B - c_A).dot(normal);
            }
        }
        break;
        case b3Manifold::e_face_B: {

            normal = xf_B.rotate(manifold->m_local_normal);
            b3Vec3r plane_point = xf_B.transform(manifold->m_local_point);

            for (int i = 0; i < manifold->m_point_count; ++i) {
                b3Vec3r clip_point = xf_A.transform(manifold->m_points[i].m_local_point);
                b3Vec3r c_B = clip_point + (radius_B - (clip_point - plane_point).dot(normal)) * normal;
                b3Vec3r c_A = clip_point - radius_A * normal;
                points[i] = (c_A + c_B) * real(0.5);
                separations[i] = (c_B - c_A).dot(normal);
            }
        }
        break;
    }
}

