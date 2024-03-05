
#include "collision/b3_collision.hpp"
#include "geometry/b3_plane_shape.hpp"
#include "geometry/b3_sphere_shape.hpp"

#include "math/b3_math.hpp"


void b3_collide_plane_and_sphere(b3Manifold* manifold,
                                 const b3PlaneShape* plane_a,
                                 const b3TransformD& xf_a,
                                 const b3SphereShape* sphere_b,
                                 const b3TransformD& xf_b) {
    manifold->m_point_count = 0;

    // transform sphere center to plane frame
    b3Vector3d local_center = xf_b.transform(sphere_b->get_centroid());
    local_center = xf_a.transform_local(local_center);

    b3Vector3d nearest_point;
    if(local_center.x() < -plane_a->m_half_width) {
        nearest_point[0] = -plane_a->m_half_width;
    } else if(local_center.x() > plane_a->m_half_width) {
        nearest_point[0] = plane_a->m_half_width;
    } else {
        nearest_point[0] = local_center.x();
    }

    if(local_center.y() < -plane_a->m_half_length) {
        nearest_point[1] = -plane_a->m_half_length;
    } else if(local_center.y() > plane_a->m_half_length) {
        nearest_point[1] = plane_a->m_half_length;
    } else {
        nearest_point[1] = local_center.y();
    }
    double sq_distance = (local_center - nearest_point).length2();
    double radius = sphere_b->get_radius() + plane_a->get_radius();
    if(sq_distance > radius * radius) {
        return;
    }
    manifold->m_point_count = 1;
    manifold->m_local_normal = xf_a.transform(local_center - nearest_point);
    if(manifold->m_local_normal.is_zero()) {
        manifold->m_local_normal = b3Vector3d(0, 0, 1);
    } else {
        manifold->m_local_normal = manifold->m_local_normal.normalized();
    }
    manifold->m_penetration = (b3_sqrt(sq_distance) - radius) / 2.0;
    manifold->m_points[0].m_local_point = xf_a.transform(nearest_point) +
                                          manifold->m_local_normal * manifold->m_penetration;
}
