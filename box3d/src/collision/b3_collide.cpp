
#include "collision/b3_collision.hpp"


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

            real outer_radius = radius_A > radius_B ? radius_A : radius_B;
            real inner_radius = radius_A > radius_B ? radius_B : radius_A;
            b3Vec3r outer_point = radius_A > radius_B ? point_A : point_B;
            b3Vec3r inner_point = radius_A > radius_B ? point_B : point_A;

            if ((point_A - point_B).length2() < outer_radius * outer_radius && (point_A - point_B).length2() > inner_radius * inner_radius) {
                normal = (outer_point - inner_point).normalized();
                b3Vec3r outer_p = outer_point - normal * outer_radius;
                b3Vec3r inner_p = inner_point - normal * inner_radius;
                points[0] = (inner_p + outer_p) * real(0.5);
                separations[0] = -(outer_point - inner_point).dot(normal) + outer_radius - inner_radius;
                normal = (inner_point - outer_point).normalized();
            }

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

