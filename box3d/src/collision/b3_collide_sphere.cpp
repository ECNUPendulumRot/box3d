
#include "collision/b3_collision.hpp"
#include "geometry/b3_sphere_shape.hpp"
#include "geometry/b3_cube_shape.hpp"

#include "math/b3_math_op.hpp"

void b3_collide_spheres(
    b3Manifold* manifold,
    const b3SphereShape* sphere_a,
    const b3Transr& xf_a,
    const b3SphereShape* sphere_b,
    const b3Transr& xf_b)
{
    
    manifold->m_point_count = 0;
    // the center of sphere A and sphere B in world frame.
    b3Vec3r ca = xf_a.transform(sphere_a->get_centroid());
    b3Vec3r cb = xf_b.transform(sphere_b->get_centroid());

    b3Vec3r ab = cb - ca;

    real sq_distance = ab.length2();
    real radius = sphere_a->get_radius() + sphere_b->get_radius();

    if (sq_distance > radius * radius) {
        // two spheres are not collide.
        return;
    }

    manifold->m_point_count = 1;

    // if the vector ab is nearly zero, we select the x-axis as the normal
    if(ab.is_zero()) {
        manifold->m_local_normal = b3Vec3r(1.0, 0, 0);
    } else {
        manifold->m_local_normal = ab.normalized();
    }

    manifold->m_local_point = sphere_a->get_centroid();
    manifold->m_points[0].m_local_point = sphere_b->get_centroid();
    manifold->m_type = b3Manifold::e_spheres;
}


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

    if (normal.length2() < b3_real_epsilon) {
        find_normal(cube_a, sphere_c_local, normal, closest_point);
    }

    manifold->m_point_count = 1;
    manifold->m_local_normal = normal.normalized();
    manifold->m_local_point = closest_point;
    manifold->m_type = b3Manifold::e_face_A;
    manifold->m_points[0].m_local_point = {0, 0, 0};
}

